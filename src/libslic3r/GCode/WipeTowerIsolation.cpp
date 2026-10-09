// Prime tower material isolation: each filament adhesiveness category owns a walled section of
// the tower, printed only by filaments of that category, so dissimilar materials never stack.
//
// This file holds the isolation-specific bodies for WipeTower2 and ToolOrdering. The call sites in
// WipeTower2.cpp and ToolOrdering.cpp are kept to a few lines each so that the feature stays easy
// to carry across upstream changes to those files. See docs/prime_tower_isolation_design.md.

#include "libslic3r.h"
#include "ClipperUtils.hpp"
#include "ExtrusionEntity.hpp"
#include "Polygon.hpp"
#include "Print.hpp"
#include "PrintConfig.hpp"
#include "ToolOrdering.hpp"
#include "WipeTower2.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <map>
#include <set>

namespace Slic3r {

// Defined in WipeTower2.cpp.
Polylines contrust_gap_for_skip_points(
    const Polygon& polygon, const std::vector<Vec2f>& skip_points, float wt_width, float gap_length, Polygon& insert_skip_polygon);
Polygon generate_rectange_polygon(const Vec2f& wt_box_min, const Vec2f& wt_box_max);

// ---------------------------------------------------------------------------------------------
// WipeTower2
// ---------------------------------------------------------------------------------------------

std::vector<WipeTower2::Section> WipeTower2::get_sections() const
{
    std::vector<Section> out;
    for (const auto& block : m_wipe_tower_blocks)
        out.push_back({block.filament_adhesiveness_category, block.start_depth, block.start_depth + block.depth});
    return out;
}

// Called from plan_tower_new() before the wall type is used. Isolation draws one wall per
// section. Rectangle and rib walls work per section (a rib wall is the footprint rectangle plus
// a fin at each corner, and each fin goes to the section that owns the corner). The cone is a
// skirt printed on every layer by that layer's tool, which would stack categories like the shared
// wall did, so it falls back to rectangle (validate() warns). The internal-ribs depth copy (which
// would extend a section past its last use) does not apply. With a single category there is only
// one section and the tower is printed as configured (see isolation_active()), so decide from the
// categories the toolchanges actually use.
void WipeTower2::isolation_plan_wall_type()
{
    std::set<int> categories;
    for (const WipeTowerInfo& layer : m_plan) {
        for (const auto* changes : {&layer.tool_changes, &layer.local_z_tool_changes})
            for (const WipeTowerInfo::ToolChange& tc : *changes)
                if (tc.old_tool != tc.new_tool) {
                    categories.insert(get_filament_category(tc.old_tool));
                    categories.insert(get_filament_category(tc.new_tool));
                }
    }
    if (categories.size() > 1) {
        if (m_wall_type == (int) WipeTowerWallType::wtwCone)
            m_wall_type = (int) WipeTowerWallType::wtwRectangle;
        m_tower_framework = false;
    }
}

// Called from plan_tower_new() in the rib branch, which squares the footprint from the sum of
// the section depths: under isolation the footprint also carries the spacing between sections.
float WipeTower2::isolation_extra_depth() const
{
    return isolation_active() ? section_spacing() * float(m_wipe_tower_blocks.size() - 1) : 0.f;
}

// Called from generate_wipe_tower_blocks() after the per-layer depths are known. A section's
// depth on a layer is what its purges there and above need (the caller has already made the
// depths non-increasing upwards, as for the stock tower), at least m_min_section_depth; the section
// ends where its category is no longer active. With rib walls the fins brace a section at its
// corners, so every section keeps its largest depth instead (the stock rib tower keeps its full
// footprint too). block.depth, the largest of these, lays out the footprint, so a section that
// shrinks leaves a gap towards the next one and never moves it. Each layer's depth also carries
// the spacing between the sections active on it. Depths are aligned to the line-width grid so that
// align_perimeter() leaves the section walls where they are laid out; otherwise it would push
// neighbouring walls into each other and close the air gap.
void WipeTower2::isolation_layout_blocks()
{
    auto aligned = [this](float depth) { return std::ceil(std::max(depth, m_min_section_depth) / m_perimeter_width) * m_perimeter_width; };
    for (auto& block : m_wipe_tower_blocks)
        block.depth = aligned(block.depth);
    for (size_t layer_id = 0; layer_id < m_plan.size(); ++layer_id) {
        int active = 0;
        m_plan[layer_id].depth = 0;
        for (auto& block : m_wipe_tower_blocks) {
            if (block.layer_depths[layer_id] > WT_EPSILON) {
                block.layer_depths[layer_id] = m_use_rib_wall ? block.depth : aligned(block.layer_depths[layer_id]);
                ++active;
            }
            m_plan[layer_id].depth += block.layer_depths[layer_id];
        }
        if (active > 1)
            m_plan[layer_id].depth += section_spacing() * float(active - 1);
    }
}

// Called from generate_new() on a sparse layer (no toolchange). Only the current tool's category
// can be active here, so every active section (its wall and fill) is printed with the current
// tool. Returns one result per active section; the caller merges them and appends the brim pass.
std::vector<WipeTower::ToolChangeResult> WipeTower2::isolation_finish_sparse_sections(bool extrude_fill)
{
    std::vector<WipeTower::ToolChangeResult> out;
    for (WipeTowerBlock& block : m_wipe_tower_blocks) {
        if (block.layer_depths[m_cur_layer_id] <= WT_EPSILON) {
            block.finish_depth[m_cur_layer_id] = block.start_depth;
            continue;
        }
        assert(block.filament_adhesiveness_category == get_filament_category(m_current_tool));
        out.emplace_back(finish_block(block, m_current_tool, extrude_fill));
        block.finish_depth[m_cur_layer_id] = block.cur_depth;
    }
    return out;
}

// Called from generate_new() when choosing which filament finishes a section on a toolchange
// layer. Only the section's own category may print it. Prefer the filament that purged into it
// (candidate); otherwise any filament of its category present on this layer, which ToolOrdering
// guarantees exists while the section is active.
int WipeTower2::isolation_finish_filament(const WipeTowerBlock& block, const WipeTowerInfo& layer, int candidate, int fallback)
{
    int filament = candidate;
    if (filament != -1 && get_filament_category(filament) != block.filament_adhesiveness_category)
        filament = -1;
    if (filament == -1) {
        for (const auto& tc : layer.tool_changes) {
            if (get_filament_category(int(tc.new_tool)) == block.filament_adhesiveness_category) { filament = int(tc.new_tool); break; }
            if (get_filament_category(int(tc.old_tool)) == block.filament_adhesiveness_category) { filament = int(tc.old_tool); break; }
        }
    }
    assert(filament != -1 && "isolation: no filament of the section's category on this layer");
    if (filament == -1)
        filament = fallback; // keep the output valid in release builds
    return filament;
}

// Called from finish_block(). This section's own wall replaces the shared outer wall. It sits at
// the same X as the old outer wall, one line below and at the end of this section's interior (the
// same convention the shared wall used for the whole tower). With rib walls the outline is the
// whole tower's rib outline clipped to this section's band, so the first section carries the two
// front fins and the last section the two back fins, each printed as part of that section's loop.
// Leaves the entry gap at this layer's toolchange entry points that lie on this section's side
// walls, exactly as the shared outer wall did. Returns the paths to extrude; wall_box receives the
// wall rectangle and the closed wall outline is recorded in m_outer_wall.
Polylines WipeTower2::isolation_section_wall(const WipeTowerBlock& block, WipeTower::box_coordinates& wall_box)
{
    wall_box = WipeTower::box_coordinates(
        Vec2f(0.f, block.start_depth - m_perimeter_width),
        m_wipe_tower_width,
        block.layer_depths[m_cur_layer_id] + m_perimeter_width);
    wall_box = align_perimeter(wall_box);
    Polygon wall_polygon = generate_rectange_polygon(wall_box.ld, wall_box.ru);
    if (m_use_rib_wall) {
        // Clip the whole tower's rib outline to this section: its own band, extended outwards past
        // the footprint on the sides where it owns the corners.
        WipeTower::box_coordinates tower_box = align_perimeter(WipeTower::box_coordinates(Vec2f(0.f, 0.f), m_wipe_tower_width, m_wipe_tower_depth));
        Polygon rib_polygon = generate_rib_polygon(tower_box);
        const bool  first = &block == &m_wipe_tower_blocks.front();
        const bool  last  = &block == &m_wipe_tower_blocks.back();
        const float far   = m_rib_length + m_wipe_tower_width + m_wipe_tower_depth;
        Polygon band = generate_rectange_polygon(
            Vec2f(-far, first ? -far : wall_box.ld.y()),
            Vec2f(m_wipe_tower_width + far, last ? m_wipe_tower_depth + far : wall_box.lu.y()));
        Polygons clipped = intersection(Polygons{rib_polygon}, Polygons{band});
        if (!clipped.empty()) {
            wall_polygon = *std::max_element(clipped.begin(), clipped.end(), [](const Polygon& a, const Polygon& b) { return std::abs(a.area()) < std::abs(b.area()); });
            if (m_used_fillet) // round the fins, keep the rectangle's corners (as generate_support_wall_new does)
                wall_polygon = union_({rounding_polygon(wall_polygon), generate_rectange_polygon(wall_box.ld, wall_box.ru)}).front();
        }
    }
    std::vector<Vec2f> skip_points;
    if (m_use_gap_wall && m_cur_layer_id >= 0 && size_t(m_cur_layer_id) < m_wall_skip_points.size()) {
        for (const Vec2f& pt : m_wall_skip_points[m_cur_layer_id])
            if (pt.y() >= wall_box.ld.y() - WT_EPSILON && pt.y() <= wall_box.lu.y() + WT_EPSILON)
                skip_points.push_back(pt);
    }
    Polygon   insert_skip_polygon;
    Polylines wall_paths = contrust_gap_for_skip_points(wall_polygon, skip_points, m_wipe_tower_width, 2.5f * m_perimeter_width, insert_skip_polygon);
    Polyline wall_line = to_polyline(insert_skip_polygon);
    wall_line.translate(0, scaled(m_y_shift));
    m_outer_wall[m_z_pos].push_back(wall_line);
    isolation_section_brim(block, wall_polygon, wall_paths);
    return wall_paths;
}

// Called from isolation_section_wall(). Each section has its own brim, printed by its own
// material right after its wall: the loops the stock tower would draw around the whole footprint
// (first-layer brim, then the chamfer that tapers it into the wall over the next few layers when
// prime_tower_brim_chamfer is on) are drawn around this section's outline and clipped at the
// midline of the gap on each side it shares with a neighbour. The sections are spaced so that
// the innermost loop of each goes all the way around (see section_spacing()); the outer loops
// are cut at the midline and only continue outside the footprint.
void WipeTower2::isolation_section_brim(const WipeTowerBlock& block, const Polygon& outline, Polylines& paths)
{
    const float spacing   = m_perimeter_width - m_layer_height * float(1. - M_PI_4);
    int         loops_num = int((m_wipe_tower_brim_width + spacing / 2.f) / spacing);
    const bool  first_layer = is_first_layer();
    if (!first_layer) {
        const int layer_idx = int(m_layer_info - m_plan.begin());
        if (!m_prime_tower_brim_chamfer || m_first_layer_idx == size_t(-1) || m_first_layer_idx >= m_plan.size())
            loops_num = 0;
        else if (block.layer_depths[layer_idx] != block.layer_depths[m_first_layer_idx])
            loops_num = 0; // stop the chamfer if the section's depth changes (as the stock tower does for the whole tower)
        else {
            const float max_chamfer_width = 3.f;
            loops_num = std::min(loops_num, int(max_chamfer_width / spacing)) - (layer_idx - int(m_first_layer_idx));
        }
    }
    if (loops_num <= 0)
        return;

    // This section's band: from the gap midline below to the gap midline above, open on the
    // sides where it owns the footprint edge. Midlines come from the laid-out (largest) depths so
    // that neighbouring bands always meet, whatever a section's depth is on this layer.
    const bool  first = &block == &m_wipe_tower_blocks.front();
    const bool  last  = &block == &m_wipe_tower_blocks.back();
    const float far   = m_wipe_tower_brim_width + m_rib_length + m_wipe_tower_width + m_wipe_tower_depth;
    const float half_gap = (section_spacing() - m_perimeter_width) / 2.f; // wall centre to midline
    const float low  = first ? -far : block.start_depth - m_perimeter_width - half_gap;
    const float high = last ? m_wipe_tower_depth + far : block.start_depth + block.depth + half_gap;
    Polygon band = generate_rectange_polygon(Vec2f(-far, low), Vec2f(m_wipe_tower_width + far, high));

    Polygon loop = outline;
    for (int i = 0; i < loops_num; ++i) {
        loop = offset(loop, scaled(spacing)).front();
        Polyline closed = to_polyline(loop);
        closed.points.push_back(closed.points.front());
        for (Polyline& part : intersection_pl(Polylines{closed}, band)) {
            Polyline recorded = part;
            recorded.translate(0, scaled(m_y_shift));
            m_outer_wall[m_z_pos].push_back(recorded);
            paths.push_back(std::move(part));
        }
    }
    if (first_layer) // actual brim width, for the skirt and the preview box (as finish_layer_new does)
        m_wipe_tower_brim_width_real = loops_num * spacing + spacing / 2.f;
}

// Called from finish_layer_new() on the first layer. The stock tower records how far the rib
// fins reach past the origin so the whole tower is shifted to keep them inside its placement;
// under isolation the brim-only pass gets the same outline, so record it from there.
void WipeTower2::isolation_set_rib_offset(const Polygon& outer_wall)
{
    if (m_use_rib_wall) {
        BoundingBox bbox = get_extents(outer_wall);
        m_rib_offset = Vec2f(-unscaled<float>(bbox.min.x()), -unscaled<float>(bbox.min.y()));
    } else
        m_rib_offset = Vec2f(0.f, 0.f); // rectangle walls start at the tower origin
}

// ---------------------------------------------------------------------------------------------
// ToolOrdering
// ---------------------------------------------------------------------------------------------

int ToolOrdering::filament_category(unsigned int extruder) const
{
    return Slic3r::filament_category(*m_print_config_ptr, extruder);
}

// Called from insert_wipe_tower_extruder(). Every adhesiveness category still active at this
// height (used on this layer or on any layer above) needs a filament of that category on this
// layer, so that its section of the tower is printed by its own material. Walk top-down and
// remember the filament seen nearest above for each category; that is the one inserted, so two
// spools of the same material never ping-pong. Only WipeTower2 (non-BBL printers) prints isolated
// sections, so the BBL tower gets no extra toolchanges.
bool ToolOrdering::insert_isolation_extruders()
{
    if (!m_print_config_ptr->prime_tower_isolate_materials || m_is_BBL_printer)
        return false;
    bool changed = false;
    std::map<int, unsigned int> representative; // category -> filament seen nearest above
    for (int i = int(m_layer_tools.size()) - 1; i >= 0; --i) {
        LayerTools& lt = m_layer_tools[i];
        if (lt.extruders.empty())
            continue;
        std::set<int> present;
        for (unsigned int extruder : lt.extruders) {
            int category = filament_category(extruder);
            present.insert(category);
            representative[category] = extruder;
        }
        if (!lt.has_wipe_tower)
            continue;
        bool layer_changed = false;
        for (const auto& [category, extruder] : representative) {
            if (present.count(category) == 0) {
                lt.extruders.emplace_back(extruder);
                layer_changed = true;
            }
        }
        if (layer_changed) {
            sort_remove_duplicates(lt.extruders);
            changed = true;
        }
    }
    return changed;
}

} // namespace Slic3r
