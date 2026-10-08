#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "test_utils.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/GCode/ToolOrdering.hpp"

#include <algorithm>
#include <limits>
#include <cmath>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <boost/filesystem.hpp>
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

// Two physical filaments: 0 = PLA (category 100), 1 = PETG (category 300).
static DynamicPrintConfig two_material_config(bool isolate)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_extruders(2);
    config.set_num_filaments(2);
    config.option<ConfigOptionFloats>("filament_diameter")->values = {1.75, 1.75};
    config.option<ConfigOptionStrings>("filament_colour")->values = {"#FF0000", "#0000FF"};
    config.option<ConfigOptionInts>("filament_adhesiveness_category")->values = {100, 300};
    config.set("enable_prime_tower", true);
    config.set("prime_tower_isolate_materials", isolate);
    config.set("prime_tower_width", 30.);
    config.set("layer_height", 0.2);
    config.set("initial_layer_print_height", 0.2);
    config.set("enable_support", false);
    config.set("skirt_loops", 0);
    config.set("layer_change_gcode", "G92 E0"); // required by Print::validate with relative E
    // Print::validate keeps only the last warning it finds; keep the motion-ability checks quiet
    // so the isolation warning is observable.
    config.option<ConfigOptionFloats>("machine_max_acceleration_extruding")->values = {100000., 100000.};
    config.option<ConfigOptionFloats>("machine_max_acceleration_travel")->values = {100000., 100000.};
    config.option<ConfigOptionFloats>("machine_max_jerk_x")->values = {1000., 1000.};
    config.option<ConfigOptionFloats>("machine_max_jerk_y")->values = {1000., 1000.};
    return config;
}

// A 4 mm PLA cube next to a PETG object made of a 1 mm bottom slab and a 1 mm slab floating
// above a gap (no support). add_volume centres a mesh on its origin, so the floating slab
// with a 3 mm offset spans z = 2.5..3.5. Observed tower layers (19; the top object layer
// carries no tower): indices 0..4 both materials, 5..12 PLA only with PETG still active
// above (where isolation must insert PETG), 13..16 both, 17..18 PLA only with nothing above.
static constexpr size_t GAP_FIRST = 5;
static constexpr size_t GAP_LAST  = 12;
static constexpr size_t TOP_FIRST = 17;
static constexpr size_t TOWER_LAYERS = 19;

static void add_gap_objects(Model& model, int pla_extruder = 1, int petg_extruder = 2)
{
    ModelObject* pla = model.add_object();
    pla->add_volume(make_cube(10., 10., 4.));
    pla->add_instance();
    pla->config.set("extruder", pla_extruder);
    pla->instances.front()->set_offset(Vec3d(50., 50., 0.));

    ModelObject* petg = model.add_object();
    petg->add_volume(make_cube(10., 10., 1.));
    ModelVolume* top = petg->add_volume(make_cube(10., 10., 1.));
    top->set_offset(Vec3d(0., 0., 3.));
    petg->add_instance();
    petg->config.set("extruder", petg_extruder);
    petg->instances.front()->set_offset(Vec3d(80., 50., 0.));
}

static std::vector<unsigned int> sorted_copy(std::vector<unsigned int> v)
{
    std::sort(v.begin(), v.end());
    return v;
}

static std::vector<std::vector<unsigned int>> extruders_per_tower_layer(const Print& print)
{
    std::vector<std::vector<unsigned int>> out;
    for (const LayerTools& lt : print.get_tool_ordering())
        if (lt.has_wipe_tower)
            out.push_back(lt.extruders);
    return out;
}

} // namespace

TEST_CASE("Prime tower isolation options exist with documented defaults", "[PrimeTowerIsolation]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    REQUIRE(config.has("prime_tower_isolate_materials"));
    REQUIRE(config.has("prime_tower_min_section_depth"));
    CHECK(config.opt_bool("prime_tower_isolate_materials") == false);
    CHECK_THAT(config.opt_float("prime_tower_min_section_depth"), WithinAbs(10., 1e-9));
}

TEST_CASE("Prime tower isolation options belong to the print preset", "[PrimeTowerIsolation]")
{
    const std::vector<std::string>& keys = Preset::print_options();
    const bool has_isolate = std::find(keys.begin(), keys.end(), "prime_tower_isolate_materials") != keys.end();
    const bool has_depth   = std::find(keys.begin(), keys.end(), "prime_tower_min_section_depth") != keys.end();
    CHECK(has_isolate);
    CHECK(has_depth);
}

TEST_CASE("Isolation inserts the missing category on single-material layers", "[PrimeTowerIsolation]")
{
    Model model;
    add_gap_objects(model);

    SECTION("off: gap layers use PLA only") {
        Print print;
        print.set_status_silent();
        print.apply(model, two_material_config(false));
        print.process();
        auto layers = extruders_per_tower_layer(print);
        REQUIRE(layers.size() == TOWER_LAYERS);
        for (size_t i = GAP_FIRST; i <= GAP_LAST; ++i) {
            CAPTURE(i);
            CHECK(layers[i] == std::vector<unsigned int>{0});
        }
    }

    SECTION("on: gap layers gain PETG because it is still active above") {
        Print print;
        print.set_status_silent();
        print.apply(model, two_material_config(true));
        print.process();
        auto layers = extruders_per_tower_layer(print);
        REQUIRE(layers.size() == TOWER_LAYERS);
        for (size_t i = 0; i < TOP_FIRST; ++i) {
            CAPTURE(i);
            CHECK(sorted_copy(layers[i]) == std::vector<unsigned int>{0, 1});
        }
        // Above the top PETG slab nothing uses PETG, so its section has ended: no insertion.
        for (size_t i = TOP_FIRST; i < layers.size(); ++i) {
            CAPTURE(i);
            CHECK(layers[i] == std::vector<unsigned int>{0});
        }
    }

    SECTION("on with a BBL printer: the BBL tower cannot isolate, so nothing is inserted") {
        Print print;
        print.set_status_silent();
        print.is_BBL_printer() = true;
        print.apply(model, two_material_config(true));
        print.process();
        auto layers = extruders_per_tower_layer(print);
        REQUIRE(layers.size() == TOWER_LAYERS);
        for (size_t i = GAP_FIRST; i <= GAP_LAST; ++i) {
            CAPTURE(i);
            CHECK(layers[i] == std::vector<unsigned int>{0});
        }
    }
}

TEST_CASE("Isolation does not insert a category that is no longer used above", "[PrimeTowerIsolation]")
{
    // Swap roles: the gapped object is extruder 1 (category 100), the solid cube extruder 2
    // (category 300). The gap gets extruder 1 inserted because it is used above; the layers
    // above the top slab use only extruder 2 and nothing of category 100 is above them.
    Model model;
    add_gap_objects(model, /*pla_extruder=*/2, /*petg_extruder=*/1);
    Print print;
    print.set_status_silent();
    print.apply(model, two_material_config(true));
    print.process();
    auto layers = extruders_per_tower_layer(print);
    REQUIRE(layers.size() == TOWER_LAYERS);
    for (size_t i = 0; i < TOP_FIRST; ++i) {
        CAPTURE(i);
        CHECK(sorted_copy(layers[i]) == std::vector<unsigned int>{0, 1});
    }
    for (size_t i = TOP_FIRST; i < layers.size(); ++i) {
        CAPTURE(i);
        CHECK(layers[i] == std::vector<unsigned int>{1});
    }
}

TEST_CASE("Isolation inserts nothing when every filament shares a category", "[PrimeTowerIsolation]")
{
    Model model;
    add_gap_objects(model);
    DynamicPrintConfig off = two_material_config(false);
    DynamicPrintConfig on  = two_material_config(true);
    off.option<ConfigOptionInts>("filament_adhesiveness_category")->values = {100, 100};
    on.option<ConfigOptionInts>("filament_adhesiveness_category")->values  = {100, 100};

    Print print_off, print_on;
    print_off.set_status_silent();
    print_on.set_status_silent();
    print_off.apply(model, off);
    print_off.process();
    print_on.apply(model, on);
    print_on.process();
    auto layers = extruders_per_tower_layer(print_on);
    REQUIRE(layers.size() == TOWER_LAYERS);
    CHECK(extruders_per_tower_layer(print_off) == layers);
    for (size_t i = GAP_FIRST; i <= GAP_LAST; ++i) {
        CAPTURE(i);
        CHECK(layers[i] == std::vector<unsigned int>{0});
    }
}

TEST_CASE("Isolation prefers the spool seen nearest above when two spools share a category", "[PrimeTowerIsolation]")
{
    // Three filaments: 0 = PLA-A (100), 1 = PLA-B (100), 2 = PETG (300).
    // Solid 4 mm PETG cube; a second object whose bottom slab is PLA-A and whose floating
    // top slab is PLA-B (per-volume extruders). In the gap (PETG-only layers) the PLA
    // section is active because PLA-B is used above, and the representative must be PLA-B.
    DynamicPrintConfig config = two_material_config(true);
    config.set_num_extruders(3);
    config.set_num_filaments(3);
    config.option<ConfigOptionFloats>("filament_diameter")->values = {1.75, 1.75, 1.75};
    config.option<ConfigOptionStrings>("filament_colour")->values = {"#FF0000", "#00FF00", "#0000FF"};
    config.option<ConfigOptionInts>("filament_adhesiveness_category")->values = {100, 100, 300};

    Model model;
    ModelObject* petg = model.add_object();
    petg->add_volume(make_cube(10., 10., 4.));
    petg->add_instance();
    petg->config.set("extruder", 3);
    petg->instances.front()->set_offset(Vec3d(50., 50., 0.));

    ModelObject* pla = model.add_object();
    ModelVolume* bottom = pla->add_volume(make_cube(10., 10., 1.));
    bottom->config.set("extruder", 1);
    ModelVolume* top = pla->add_volume(make_cube(10., 10., 1.));
    top->set_offset(Vec3d(0., 0., 3.));
    top->config.set("extruder", 2);
    pla->add_instance();
    pla->config.set("extruder", 1);
    pla->instances.front()->set_offset(Vec3d(80., 50., 0.));

    Print print;
    print.set_status_silent();
    print.apply(model, config);
    print.process();
    auto layers = extruders_per_tower_layer(print);
    REQUIRE(layers.size() == TOWER_LAYERS);
    for (size_t i = GAP_FIRST; i <= GAP_LAST; ++i) {
        CAPTURE(i);
        CHECK(sorted_copy(layers[i]) == std::vector<unsigned int>{1, 2});
    }
    for (size_t i = TOP_FIRST; i < layers.size(); ++i) {
        CAPTURE(i);
        CHECK(layers[i] == std::vector<unsigned int>{2});
    }
}

TEST_CASE("Isolation treats filaments beyond the category list as category 0", "[PrimeTowerIsolation]")
{
    Model model;
    add_gap_objects(model);
    DynamicPrintConfig config = two_material_config(true);
    config.option<ConfigOptionInts>("filament_adhesiveness_category")->values = {100}; // shorter than filament count
    Print print;
    print.set_status_silent();
    print.apply(model, config);
    REQUIRE_NOTHROW(print.process());
    // Categories are {100, 0}: still two categories, so PETG is inserted on the gap layers.
    auto layers = extruders_per_tower_layer(print);
    REQUIRE(layers.size() == TOWER_LAYERS);
    for (size_t i = GAP_FIRST; i <= GAP_LAST; ++i) {
        CAPTURE(i);
        CHECK(sorted_copy(layers[i]) == std::vector<unsigned int>{0, 1});
    }
}

TEST_CASE("Isolated sections are at least the minimum depth and separated by a gap", "[PrimeTowerIsolation]")
{
    Model model;
    add_gap_objects(model);
    DynamicPrintConfig config = two_material_config(true);
    config.set("prime_tower_min_section_depth", 12.);
    Print print;
    print.set_status_silent();
    print.apply(model, config);
    print.process();

    const WipeTowerData& data = print.wipe_tower_data();
    REQUIRE(data.sections.size() == 2);
    for (const WipeTower2::Section& s : data.sections) {
        CAPTURE(s.category, s.y_start, s.y_end);
        CHECK(s.y_end - s.y_start >= 12.f - 0.01f);
    }
    // Sections are ordered by y and do not overlap. Between two interiors there are two wall
    // lines plus one line of air, so the spacing is at least three line widths; with the
    // default config the tower line width is about 0.45 mm.
    REQUIRE(data.sections[0].y_end < data.sections[1].y_start);
    CHECK(data.sections[1].y_start - data.sections[0].y_end >= 3.f * 0.4f);
    CHECK(data.sections[1].y_start - data.sections[0].y_end <= 3.f * 0.7f);
    // Total depth covers both sections.
    CHECK(data.depth >= (data.sections[1].y_end - data.sections[0].y_start) - 0.01f);
}

namespace {
static int category_of(const Print& print, unsigned int tool)
{
    const auto& v = print.config().filament_adhesiveness_category.values;
    return tool < v.size() ? v[tool] : 0;
}

static const WipeTower2::Section& section_for(const WipeTowerData& data, int category)
{
    for (const auto& s : data.sections)
        if (s.category == category)
            return s;
    FAIL("no section for category " << category);
    return data.sections.front();
}
} // namespace

TEST_CASE("Isolated sections are only extruded by their own category", "[PrimeTowerIsolation]")
{
    Model model;
    add_gap_objects(model);
    Print print;
    print.set_status_silent();
    print.apply(model, two_material_config(true));
    print.process();

    const WipeTowerData& data = print.wipe_tower_data();
    REQUIRE(data.sections.size() == 2);
    const float wall_tol = 0.5f;  // a section's own wall line sits one line width outside its interior
    const float brim_tol = 0.3f;  // brim loops start one line spacing beyond the footprint
    float footprint_min = data.sections.front().y_start, footprint_max = data.sections.front().y_end;
    for (const auto& s : data.sections) {
        footprint_min = std::min(footprint_min, s.y_start);
        footprint_max = std::max(footprint_max, s.y_end);
    }

    size_t inside_own = 0, brim = 0;
    for (const auto& layer : data.tool_changes) {
        for (const WipeTower::ToolChangeResult& tcr : layer) {
            for (const WipeTower::Extrusion& e : tcr.extrusions) {
                if (e.width <= 0.f)
                    continue; // travel
                const int category = category_of(print, e.tool);
                const auto& own = section_for(data, category);
                CAPTURE(tcr.print_z, e.tool, e.pos.y(), own.y_start, own.y_end, footprint_min, footprint_max, data.depth);
                // Never inside another category's section interior.
                for (const auto& other : data.sections) {
                    if (other.category == category)
                        continue;
                    CAPTURE(other.category, other.y_start, other.y_end);
                    const bool inside_other = e.pos.y() > other.y_start + 0.01f && e.pos.y() < other.y_end - 0.01f;
                    CHECK_FALSE(inside_other);
                }
                // Either in its own section (plus wall) or outside the whole footprint (brim loops).
                const bool in_own = e.pos.y() >= own.y_start - wall_tol && e.pos.y() <= own.y_end + wall_tol;
                const bool outside_footprint = e.pos.y() < footprint_min - brim_tol || e.pos.y() > footprint_max + brim_tol;
                const bool placed_in_own_or_brim = in_own || outside_footprint;
                CAPTURE(in_own, outside_footprint);
                CHECK(placed_in_own_or_brim);
                if (in_own) ++inside_own;
                if (outside_footprint) ++brim;
            }
        }
    }
    CHECK(inside_own > 100);
    CHECK(brim > 0);
}

TEST_CASE("Sparse isolated layer prints only the active section with the current tool", "[PrimeTowerIsolation]")
{
    // Three filaments: 0 = PLA-A (100), 1 = PLA-B (100), 2 = PETG (300).
    // Solid 4 mm PLA-A cube; a second object with a PETG bottom slab (0..1 mm) and a PLA-B
    // floating slab (2.5..3.5 mm). Layer z = 1.2 still changes back from PETG to PLA-A; on
    // layers z = 1.4..2.4 only PLA-A prints and the only
    // category still active above is 100, which PLA-A already covers: no toolchange is
    // inserted, so these are sparse tower layers. They must contain extrusions from tool 0
    // only, inside the PLA section, and nothing in the (ended) PETG section.
    DynamicPrintConfig config = two_material_config(true);
    config.set_num_extruders(3);
    config.set_num_filaments(3);
    config.option<ConfigOptionFloats>("filament_diameter")->values = {1.75, 1.75, 1.75};
    config.option<ConfigOptionStrings>("filament_colour")->values = {"#FF0000", "#00FF00", "#0000FF"};
    config.option<ConfigOptionInts>("filament_adhesiveness_category")->values = {100, 100, 300};

    Model model;
    ModelObject* pla_a = model.add_object();
    pla_a->add_volume(make_cube(10., 10., 4.));
    pla_a->add_instance();
    pla_a->config.set("extruder", 1);
    pla_a->instances.front()->set_offset(Vec3d(50., 50., 0.));

    ModelObject* mixed = model.add_object();
    ModelVolume* bottom = mixed->add_volume(make_cube(10., 10., 1.));
    bottom->config.set("extruder", 3);
    ModelVolume* top = mixed->add_volume(make_cube(10., 10., 1.));
    top->set_offset(Vec3d(0., 0., 3.));
    top->config.set("extruder", 2);
    mixed->add_instance();
    mixed->config.set("extruder", 3);
    mixed->instances.front()->set_offset(Vec3d(80., 50., 0.));

    Print print;
    print.set_status_silent();
    print.apply(model, config);
    print.process();
    const WipeTowerData& data = print.wipe_tower_data();
    REQUIRE(data.sections.size() == 2);
    const WipeTower2::Section& pla  = section_for(data, 100);
    const WipeTower2::Section& petg = section_for(data, 300);

    size_t sparse_layers = 0;
    for (const auto& layer : data.tool_changes) {
        if (layer.empty() || layer.front().print_z < 1.3f || layer.front().print_z > 2.5f)
            continue;
        bool any_toolchange = false;
        for (const auto& tcr : layer)
            any_toolchange |= tcr.is_tool_change;
        CAPTURE(layer.front().print_z);
        CHECK_FALSE(any_toolchange);
        if (any_toolchange)
            continue;
        ++sparse_layers;
        for (const auto& tcr : layer)
            for (const WipeTower::Extrusion& e : tcr.extrusions) {
                if (e.width <= 0.f)
                    continue;
                CAPTURE(tcr.print_z, e.tool, e.pos.y(), pla.y_start, pla.y_end);
                CHECK(e.tool == 0u);
                CHECK(e.pos.y() >= pla.y_start - 1.f);
                CHECK(e.pos.y() <= pla.y_end + 1.f);
                const bool inside_petg = e.pos.y() > petg.y_start + 0.01f && e.pos.y() < petg.y_end - 0.01f;
                CHECK_FALSE(inside_petg);
            }
    }
    CHECK(sparse_layers >= 5);
}

TEST_CASE("Isolation with a single category warns and leaves the tower unchanged", "[PrimeTowerIsolation]")
{
    using Catch::Matchers::WithinAbs;
    Model model;
    add_gap_objects(model);
    DynamicPrintConfig on = two_material_config(true);
    on.option<ConfigOptionInts>("filament_adhesiveness_category")->values = {100, 100};
    SECTION("rectangle walls") { on.set_deserialize_strict("wipe_tower_wall_type", "rectangle"); }
    SECTION("rib walls and framework stay in effect") {
        on.set_deserialize_strict("wipe_tower_wall_type", "rib");
        on.set("prime_tower_enable_framework", true);
    }
    DynamicPrintConfig off = on;
    off.set("prime_tower_isolate_materials", false);

    Print print_on, print_off;
    print_on.set_status_silent();
    print_off.set_status_silent();
    print_on.apply(model, on);
    StringObjectException warning;
    const StringObjectException err = print_on.validate(&warning);
    CAPTURE(err.string, warning.string);
    CHECK(err.string.empty());
    CHECK(warning.string.find("no effect") != std::string::npos);
    print_on.process();
    print_off.apply(model, off);
    print_off.process();

    const WipeTowerData& a = print_on.wipe_tower_data();
    const WipeTowerData& b = print_off.wipe_tower_data();
    CHECK_THAT(double(a.depth), WithinAbs(double(b.depth), 1e-4));
    REQUIRE(a.tool_changes.size() == b.tool_changes.size());
    for (size_t i = 0; i < a.tool_changes.size(); ++i) {
        REQUIRE(a.tool_changes[i].size() == b.tool_changes[i].size());
        for (size_t j = 0; j < a.tool_changes[i].size(); ++j) {
            CAPTURE(i, j);
            CHECK(a.tool_changes[i][j].gcode == b.tool_changes[i][j].gcode);
        }
    }
}

TEST_CASE("Isolated tower reports its full footprint up to the top", "[PrimeTowerIsolation]")
{
    // The PETG section (nearer y=0 or not, depending on block order) ends below the PLA section;
    // a shrinking centred box would no longer cover the section still printing.
    Model model;
    add_gap_objects(model);
    Print print;
    print.set_status_silent();
    print.apply(model, two_material_config(true));
    print.process();
    const WipeTowerData& data = print.wipe_tower_data();
    REQUIRE(data.z_and_depth_pairs.size() >= 2);
    for (size_t i = 0; i + 1 < data.z_and_depth_pairs.size(); ++i) {
        CAPTURE(i, data.z_and_depth_pairs[i].first);
        CHECK_THAT(double(data.z_and_depth_pairs[i].second), WithinAbs(double(data.depth), 1e-4));
    }
    CHECK_THAT(double(data.z_and_depth_pairs.back().first), WithinAbs(double(data.height), 1e-4));
    CHECK_THAT(double(data.z_and_depth_pairs.back().second), WithinAbs(0., 1e-6));
}

TEST_CASE("Isolated prime tower exports G-code with more toolchanges and unchanged X extents", "[PrimeTowerIsolation]")
{
    using Catch::Matchers::WithinAbs;
    Model model;
    add_gap_objects(model);
    auto run = [&](bool isolate) {
        CAPTURE(isolate);
        Print print;
        print.set_status_silent();
        DynamicPrintConfig config = two_material_config(isolate);
        config.set("prime_tower_min_section_depth", 15.);
        print.apply(model, config);
        print.process();
        boost::filesystem::path path = boost::filesystem::temp_directory_path() /
            boost::filesystem::unique_path("isolation-%%%%-%%%%.gcode");
        GCodeProcessorResult result;
        const std::string output_path = print.export_gcode(path.string(), &result, nullptr);
        REQUIRE(!output_path.empty());
        std::ifstream in(output_path);
        std::string gcode((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        in.close();
        boost::filesystem::remove(output_path);
        REQUIRE(!gcode.empty());
        return std::make_pair(gcode, print.wipe_tower_data().bbx);
    };
    auto [gcode_off, bbx_off] = run(false);
    auto [gcode_on, bbx_on]   = run(true);

    CHECK_THAT(bbx_on.min.x(), WithinAbs(bbx_off.min.x(), 1e-3));
    CHECK_THAT(bbx_on.max.x(), WithinAbs(bbx_off.max.x(), 1e-3));
    CHECK(bbx_on.size().y() > bbx_off.size().y());

    auto count = [](const std::string& s, const std::string& needle) {
        size_t n = 0, pos = 0;
        while ((pos = s.find(needle, pos)) != std::string::npos) { ++n; pos += needle.size(); }
        return n;
    };
    // Isolation adds toolchanges on the gap layers and never removes any.
    CHECK(count(gcode_on, "; CP TOOLCHANGE START") > count(gcode_off, "; CP TOOLCHANGE START"));
    CHECK(count(gcode_on, "; CP EMPTY GRID START") > 0);
}

namespace {
// Tower line width as WipeTower2 derives it (nozzle diameter times Width_To_Nozzle_Ratio).
static float tower_line_width(const Print& print) { return float(print.config().nozzle_diameter.get_at(0)) * 1.25f; }

struct WallSegment { Vec2f a, b; unsigned int tool; };

// Extruded segments (width > 0) of every tower layer, reconstructed from consecutive end positions.
static std::vector<std::vector<WallSegment>> segments_per_layer(const WipeTowerData& data)
{
    std::vector<std::vector<WallSegment>> out;
    for (const auto& layer : data.tool_changes) {
        std::vector<WallSegment> segs;
        for (const WipeTower::ToolChangeResult& tcr : layer) {
            Vec2f prev = tcr.start_pos;
            for (const WipeTower::Extrusion& e : tcr.extrusions) {
                if (e.width > 0.f)
                    segs.push_back({prev, e.pos, e.tool});
                prev = e.pos;
            }
        }
        out.push_back(std::move(segs));
    }
    return out;
}

// Horizontal wall lines (both ends at the same y) of a tool's category outside its section interior.
static std::vector<float> horizontal_wall_ys(const std::vector<WallSegment>& segs, const Print& print, const WipeTowerData& data, int category)
{
    const auto& own = section_for(data, category);
    std::vector<float> ys;
    for (const WallSegment& s : segs) {
        if (category_of(print, s.tool) != category || std::abs(s.a.y() - s.b.y()) > 0.01f || std::abs(s.a.x() - s.b.x()) < 5.f)
            continue;
        // The bottom wall sits one line below the interior, the top wall exactly at its end.
        // Brim and chamfer loops lie outside the whole footprint and are shared by design; skip them.
        const float pw = tower_line_width(print);
        float footprint_min = data.sections.front().y_start, footprint_max = data.sections.front().y_end;
        for (const auto& sec : data.sections) { footprint_min = std::min(footprint_min, sec.y_start); footprint_max = std::max(footprint_max, sec.y_end); }
        if (s.a.y() < footprint_min - pw - 0.05f || s.a.y() > footprint_max + 0.05f)
            continue;
        if (s.a.y() < own.y_start - 0.01f || s.a.y() > own.y_end - 0.01f)
            ys.push_back(s.a.y());
    }
    return ys;
}
} // namespace

TEST_CASE("Isolated section walls keep a clear gap when the section depth is not a multiple of the line width", "[PrimeTowerIsolation]")
{
    Model model;
    add_gap_objects(model);
    DynamicPrintConfig config = two_material_config(true);
    config.set("prime_tower_min_section_depth", 20.3); // above the purge-driven depth, not a multiple of the 0.5 mm line width
    Print print;
    print.set_status_silent();
    print.apply(model, config);
    print.process();
    const WipeTowerData& data = print.wipe_tower_data();
    REQUIRE(data.sections.size() == 2);
    const float pw = tower_line_width(print);
    const auto layers = segments_per_layer(data);

    size_t checked_layers = 0;
    for (size_t li = 0; li < layers.size(); ++li) {
        std::vector<float> ys_a = horizontal_wall_ys(layers[li], print, data, data.sections[0].category);
        std::vector<float> ys_b = horizontal_wall_ys(layers[li], print, data, data.sections[1].category);
        if (ys_a.empty() || ys_b.empty())
            continue; // only one section active, or brim-only layer
        ++checked_layers;
        // Each section has a wall line one line width below its interior and one at its interior end.
        for (size_t si = 0; si < 2; ++si) {
            const auto& s = data.sections[si];
            const auto& ys = si == 0 ? ys_a : ys_b;
            CAPTURE(li, si, s.y_start, s.y_end, pw, ys);
            CHECK(std::any_of(ys.begin(), ys.end(), [&](float y) { return std::abs(y - (s.y_start - pw)) < 0.05f; }));
            CHECK(std::any_of(ys.begin(), ys.end(), [&](float y) { return std::abs(y - s.y_end) < 0.05f; }));
        }
        // The nearest wall lines of the two categories are at least two line widths apart (one line of air).
        float min_dist = std::numeric_limits<float>::max();
        for (float ya : ys_a)
            for (float yb : ys_b)
                min_dist = std::min(min_dist, std::abs(ya - yb));
        CAPTURE(li, min_dist, pw);
        CHECK(min_dist >= 2.f * pw - 0.05f);
    }
    CHECK(checked_layers >= 10);
}

TEST_CASE("Isolated section wall has an entry gap where a toolchange enters it", "[PrimeTowerIsolation]")
{
    Model model;
    add_gap_objects(model);
    DynamicPrintConfig config = two_material_config(true);
    REQUIRE(config.opt_bool("wipe_tower_wall_gap"));
    Print print;
    print.set_status_silent();
    print.apply(model, config);
    print.process();
    const WipeTowerData& data = print.wipe_tower_data();
    REQUIRE(data.sections.size() == 2);
    const float pw = tower_line_width(print);
    const float width = float(print.config().prime_tower_width.value);
    const auto layers = segments_per_layer(data);

    size_t checked = 0;
    for (size_t li = 1; li < data.tool_changes.size(); ++li) { // skip the first layer (brim/adhesion rules)
        for (const WipeTower::ToolChangeResult& tcr : data.tool_changes[li]) {
            if (!tcr.is_tool_change)
                continue;
            const Vec2f entry = tcr.tool_change_start_pos;
            const bool left = std::abs(entry.x()) < width / 2.f;
            const float side_x = left ? 0.f : width;
            const int category = category_of(print, unsigned(tcr.new_tool));
            const auto& own = section_for(data, category);
            if (entry.y() < own.y_start - pw || entry.y() > own.y_end + pw)
                continue; // entry not on this section's side wall
            // Vertical wall segments of this category on the entry side must not span the entry y.
            bool covered = false;
            for (const WallSegment& s : layers[li]) {
                if (category_of(print, s.tool) != category) continue;
                if (std::abs(s.a.x() - side_x) > 0.05f || std::abs(s.b.x() - side_x) > 0.05f) continue;
                const float lo = std::min(s.a.y(), s.b.y()), hi = std::max(s.a.y(), s.b.y());
                if (hi - lo < 0.01f) continue;
                if (lo < entry.y() - 0.5f * pw && hi > entry.y() + 0.5f * pw)
                    covered = true;
            }
            CAPTURE(li, tcr.print_z, tcr.new_tool, entry.x(), entry.y(), side_x);
            CHECK_FALSE(covered);
            ++checked;
        }
    }
    CHECK(checked >= 10);
}

TEST_CASE("Isolated tower never starts a pass outside its brim", "[PrimeTowerIsolation]")
{
    // The brim-only pass used to start at the current tool's section origin plus a whole-tower
    // fill depth, which put a travel well past the brim whenever that section was not the first.
    Model model;
    add_gap_objects(model);
    Print print;
    print.set_status_silent();
    print.apply(model, two_material_config(true));
    print.process();
    const WipeTowerData& data = print.wipe_tower_data();
    REQUIRE(!data.tool_changes.empty());
    const float margin = data.brim_width + 0.01f;
    const BoundingBoxf& bbx = data.bbx;
    auto inside = [&](const Vec2f& p) {
        return p.x() >= bbx.min.x() - margin && p.x() <= bbx.max.x() + margin &&
               p.y() >= bbx.min.y() - margin && p.y() <= bbx.max.y() + margin;
    };
    for (size_t layer = 0; layer < data.tool_changes.size(); ++layer) {
        for (size_t i = 0; i < data.tool_changes[layer].size(); ++i) {
            const WipeTower::ToolChangeResult& tcr = data.tool_changes[layer][i];
            CAPTURE(layer, i, tcr.start_pos.x(), tcr.start_pos.y(), bbx.min.x(), bbx.min.y(), bbx.max.x(), bbx.max.y(), margin);
            CHECK(inside(tcr.start_pos));
            CHECK(inside(tcr.end_pos));
            // Merged results keep only the first pass's start position; the travel to a later
            // pass is a plain G1 inside the G-code, so check every XY move in it as well.
            std::istringstream in(tcr.gcode);
            std::string line;
            int bad_moves = 0;
            while (std::getline(in, line)) {
                if (line.rfind("G1", 0) != 0)
                    continue;
                size_t xi = line.find(" X"), yi = line.find(" Y");
                if (xi == std::string::npos || yi == std::string::npos)
                    continue;
                Vec2f p(std::strtof(line.c_str() + xi + 2, nullptr), std::strtof(line.c_str() + yi + 2, nullptr));
                if (!inside(p)) {
                    CAPTURE(line);
                    ++bad_moves;
                }
            }
            CHECK(bad_moves == 0);
        }
    }
}

TEST_CASE("Every layer with a toolchange has tower extrusion to purge into", "[PrimeTowerIsolation]")
{
    // Isolation inserts toolchanges on layers that had none; each of them must land on a tower
    // layer that actually extrudes, or the purge happens in mid-air.
    Model model;
    add_gap_objects(model);
    Print print;
    print.set_status_silent();
    print.apply(model, two_material_config(true));
    print.process();
    const WipeTowerData& data = print.wipe_tower_data();
    // Tower layers that really extrude something, by print_z.
    std::vector<double> extruding_z;
    for (const auto& layer : data.tool_changes) {
        bool has_toolchange = false, extruded = false;
        double z = -1.;
        for (const WipeTower::ToolChangeResult& tcr : layer) {
            z = tcr.print_z;
            has_toolchange |= tcr.is_tool_change;
            for (const WipeTower::Extrusion& e : tcr.extrusions)
                extruded |= e.width > 0.f;
        }
        CAPTURE(z, has_toolchange);
        if (has_toolchange)
            CHECK(extruded);
        if (extruded)
            extruding_z.push_back(z);
    }
    // Every layer the tool ordering prints with more than one extruder must have such a tower layer.
    size_t multi_extruder_layers = 0;
    for (const LayerTools& lt : print.get_tool_ordering()) {
        if (lt.extruders.size() < 2)
            continue;
        ++multi_extruder_layers;
        CAPTURE(lt.print_z);
        CHECK(std::any_of(extruding_z.begin(), extruding_z.end(), [&](double z) { return std::abs(z - lt.print_z) < 1e-3; }));
    }
    CHECK(multi_extruder_layers > 0);
}
