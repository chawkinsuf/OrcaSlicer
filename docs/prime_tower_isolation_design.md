# Prime Tower Material Isolation — Design

Date: 2026-10-07
Status: approved design, phase 1
Target: Snapmaker Orca (Snapmaker U1, independent toolheads)

## Problem

Multi-material prints that pair a model material with a support material
that is meant *not* to bond to it (for example PLA model, PETG support)
fail at the prime tower. The tower stacks both materials in the same
column, the weak PLA/PETG interfaces delaminate, and loose purge lines are
dragged into the print.

Snapmaker Orca's `WipeTower2` already divides the tower into one Y-band
("block") per `filament_adhesiveness_category`, and each toolchange purges
into its own category's band. Isolation still fails because:

1. One shared outer wall wraps all bands and is printed by whichever
   filament changes first on the layer (or the fixed wipe tower filament),
   so the wall alternates materials layer to layer.
2. A band's sparse fill on a layer with no toolchange onto it falls back to
   the wall filament or the currently loaded filament, so the wrong material
   is laid on top of the band.

The existing Contact/Solid layer logic and the U1 "tower interface
features" mitigate the dissimilar interface instead of removing it.

Prior art: OrcaSlicer issue #8474 "Separate prime towers for multi material
prints"; the `wipe_tower_filament` setting (single-material wall) is the
usual workaround.

## Goal

An opt-in mode in which each adhesiveness category owns a walled section of
the existing tower footprint and that section is only ever printed by
filaments of its own category, so dissimilar materials never stack
vertically. The user accepts extra toolchanges on layers that would not
otherwise use a material.

## Non-goals (phase 1)

- Separately placed towers (new placement UI, model/3MF changes).
- Circular or spiral tower geometry.
- Per-section rib or cone walls.
- Single-nozzle (ramming) printers. The design does not break them, but it
  is only validated for independent toolheads.
- Bambu printers (they use the other `WipeTower` implementation).

## Design

### 1. Settings and UI

New print settings in the Prime Tower group:

| Key | Type | Default | Label |
|-----|------|---------|-------|
| `prime_tower_isolate_materials` | bool | false | Isolate materials in prime tower |
| `prime_tower_min_section_depth` | float (mm) | 10 | Minimum section depth |

Tooltip for `prime_tower_isolate_materials`: "Give each filament
adhesiveness category its own walled section of the prime tower, printed
only by filaments of that category, so dissimilar materials never stack.
Adds a toolchange on layers that would otherwise not use a material. Prints
the brim on the first layer only. Rib walls brace the front and back
sections; cone walls are printed as rectangle walls."

Tooltip for `prime_tower_min_section_depth`: "Minimum depth of each
material section when materials are isolated. Prevents lightly used
materials from producing a thin, unstable column."

Behaviour when `prime_tower_isolate_materials` is on:

- Materials are grouped by the existing per-filament
  `filament_adhesiveness_category`. No new per-filament setting.
- `wipe_tower_wall_type`: rectangle and rib walls are drawn per section
  (section 3). Cone walls fall back to rectangle with a `validate()` warning,
  because the cone skirt is printed on every layer by that layer's tool and
  would stack categories like the shared wall did.
  `prime_tower_enable_framework` (internal ribs) is ignored: isolation uses
  its own depth rule (section 3) instead, because the internal-ribs code
  copies first-layer depths to every layer and would extend a section to the
  top of the print after its material stops being used. `ConfigManipulation`
  greys out the framework and cone-angle controls, and the U1 tower
  interface controls, while isolation is on.
- `wipe_tower_filament`, if set, is still inserted into every tower layer
  but no longer prints a wall. Its tooltip gains a sentence saying so.
- The option is hidden for Bambu printers, like `wipe_tower_no_sparse_layers`.

Both keys invalidate `psWipeTower`. They are process settings and persist
through presets and 3MF project config with no extra work.

### 2. Tool ordering (`ToolOrdering`)

Generalise `insert_wipe_tower_extruder()`:

- For each layer with a tower, compute the set of **active categories**: a
  category is active on a layer if any filament of that category is used on
  that layer or any layer above it. This matches how block depth already
  propagates downward, so a section exists exactly where its category is
  active.
- For each active category with no filament on the layer, insert a
  **representative filament**: the filament of that category used on the
  nearest layer above; fallback, the lowest-index filament of that category
  used anywhere in the print.
- Then run the existing reorder and `fill_wipe_tower_partitions` steps, as
  the current hook does, so inserted toolchanges get volumes, positions and
  partitions like any other.

Categories come from `filament_adhesiveness_category` in the print config.
Filaments sharing a category share a section.

When only one category is active, nothing is inserted and the layer is
unchanged from today.

Cost: a PLA-only layer with PETG still active above gains one PETG
toolchange plus its prime purge, and the next layer starts on PETG, so one
more toolchange to return. Accepted for the U1.

### 3. Tower geometry (`WipeTower2`)

- **Section depth.** After block depths are computed, clamp each block's
  per-layer depth to at least `prime_tower_min_section_depth` on every layer
  where the block is active. Tower depth is the sum of section depths plus
  gaps.
- **Gap.** Adjacent sections are separated by one line width of clear air
  (wall centres two line widths apart), so the two walls never fuse. Lateral
  bracing between columns is not a design requirement; if real prints show
  columns need it, the gap can become a setting later.
- **Per-section walls.** `finish_block` already draws a rectangle around
  the block's remaining area. In isolation mode that rectangle is the
  section's outer wall, inset by the gap, and the shared outer perimeter in
  `finish_layer_new` is skipped. Purge lines and sparse fill are inset by
  one more line width so they stay inside the section wall.
- **Fill filament.** A section's fill on a layer is printed by the filament
  of its own category present on that layer: the one that purged into it if
  there was a toolchange, otherwise the inserted representative. The
  fallbacks to the wall filament / currently loaded filament are removed in
  this mode. Section 2 guarantees presence; an assert guards it.
- **Section depth.** With rectangle walls, on each layer a section is as
  deep as its purges on that layer and above need (never growing upwards,
  like the stock tower), at least the minimum section depth, and it ends on
  the first layer where its category is no longer active. So a section
  shrinks once a heavily used spool of its category stops, instead of
  carrying its largest depth to the top. With rib walls every section keeps
  its largest depth while active, because the fins brace it at fixed
  corners (the stock rib tower keeps its full footprint too). The footprint
  and the start of every section come from the largest depths, so a
  shrinking section never moves its neighbours. This replaces the
  internal-ribs behaviour in isolation mode.
- **Rib walls.** The stock rib outline is the footprint rectangle plus a
  fin at each corner along the tower's diagonals, shrinking with height. Per
  section, the outline is that rib outline clipped to the section's band:
  the front section prints its rectangle plus the two front fins, the back
  section its rectangle plus the two back fins, each as one loop in its own
  material; middle sections (three or more categories) are plain
  rectangles. Fillet rounding applies to the fins as in the stock tower.
  The rib planning branch squares the footprint including the section
  spacing, and the first-layer rib offset comes from the brim pass outline.
- **Contact and solid layers.** The dissimilar-interface logic (Contact,
  Contact_UP, Solid) is skipped, since nothing dissimilar stacks.
- **First layer and brim.** Usual first-layer rules, one brim around the
  whole footprint. The brim chamfer (the stock tower tapers the brim into
  the wall over the next few layers) is forced off: it is printed by
  whichever tool finishes the layer, which under isolation may be a
  different category than the brim below it. The existing
  `prime_tower_brim_chamfer` flag (developer mode, hidden from the tab on
  main) is honoured by the block tower path again at the same time; the sync
  that introduced that path had dropped it.
- **Final purge, local-Z and nozzle-change paths** keep working: they
  already target a block by category.

### 4. G-code stage, interactions, edge cases

No changes expected in `GCode.cpp` / `WipeTowerIntegration`: each
toolchange is still one tower-local result translated to the tower
position; bounding box, detours and stats are unchanged, and inserted
toolchanges flow through the existing per-layer loop.

Interactions:

- `wipe_tower_no_sparse_layers`: a layer with no toolchange can only occur
  while a single category is active, where the tower behaves as today, so
  the option keeps its meaning.
- `wipe_tower_filament`: still inserted every layer; no longer prints a
  wall.
- Tower interface features (U1): act on contact layers, which no longer
  occur; inert while isolation is on; UI greys them out.
- Purge into objects/infill: unchanged; it just reduces what lands in the
  section.
- Wall type / internal ribs: overridden as in section 1.
- Pre-slice tower estimate: the 3D view's estimated tower box
  (`PartPlate::estimate_wipe_tower_size`) does not know about the minimum
  section depth or inserted toolchanges, so it is shallower than the real
  tower until slicing finishes and the real mesh replaces it. Accepted as
  cosmetic for phase 1.

Edge cases:

- All filaments share one category: tower identical to today; validation
  emits a one-line warning that isolation has no effect.
- A category stops mid-print: its section ends there, as blocks already
  do; remaining sections continue; minimum depth still applies.
- A category first appears mid-print: its section starts at the first
  layer (active-category propagation extends downward), filled by the
  representative filament, so no column starts mid-air.
- By-object sequential printing: tower already unsupported; nothing new.

### 5. Testing

Automated (Catch2, test-first), new file
`tests/libslic3r/test_prime_tower_isolation.cpp`, following the pattern of
`test_mixed_filament.cpp` (build a `Print` in code, run tool ordering /
`Print::process`, inspect results):

- **Tool ordering.** PLA (100) object plus a shorter PETG (300) object.
  Off: PLA-only layers above the PETG object have one extruder. On: every
  layer up to the PETG top has both; layers above have PLA only. Two PLA
  spools: representative choice does not ping-pong. Single category:
  nothing inserted.
- **Section geometry.** Run the wipe tower step and inspect
  `WipeTowerData::tool_changes`: every toolchange/fill onto a section is by
  a filament of that section's category; each section depth is at least
  the minimum; tower depth equals sections plus gaps.
- **G-code emission.** Export G-code; each tower layer contains extrusions
  from both tools; tower X extents unchanged versus isolation off; output
  still parses through the G-code processor.
- **Regression.** Existing mixed-filament and print tests pass with
  isolation off; the off path is untouched.

Manual (user, Snapmaker U1): the PLA-with-PETG-supports model that fails
today, sliced with isolation on, tower inspected in preview (seam, extra
toolchanges), then printed. This provides the PR template's screenshots and
test evidence.

## Future work (not phase 1)

- Per-section rib walls for extra stiffness.
- Let a section skip layers (lag) instead of inserting a toolchange.
- Circular tower with a spiral purge path and concentric material rings.
