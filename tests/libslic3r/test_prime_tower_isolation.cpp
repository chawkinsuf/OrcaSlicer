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
#include <tuple>
#include <optional>
#include <cstdint>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

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
    // lines plus one line of air, or two brim lines with a brim (the default), so the spacing is
    // three or four line widths; with the default config the tower line width is 0.5 mm.
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
    const float wall_tol = 1.0f;  // a section's own wall line sits one line width outside its interior, its innermost brim line one brim spacing beyond that
    const float brim_tol = 0.3f;  // brim loops start one line spacing beyond the footprint
    const float width    = float(print.config().prime_tower_width.value); // rectangle walls: the footprint's x extent
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
                // Either in its own section (plus wall and innermost brim line) or outside the whole footprint (brim loops).
                const bool in_own = e.pos.y() >= own.y_start - wall_tol && e.pos.y() <= own.y_end + wall_tol;
                // Beyond the footprint in y, or past its x edges (the outer brim loops run down the
                // sides and are cut at the gap midline, so their ends lie level with the gap).
                const bool outside_footprint = e.pos.y() < footprint_min - brim_tol || e.pos.y() > footprint_max + brim_tol ||
                                               e.pos.x() < -brim_tol || e.pos.x() > width + brim_tol;
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
        if (std::min(s.a.x(), s.b.x()) < -0.1f)
            continue; // brim and chamfer loops reach past the tower's x edges; wall lines start at x = 0
        // The bottom wall sits one line below the interior, the top wall exactly at its end.
        // Skip anything outside the whole footprint (brim and chamfer loops, rib fins).
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
    SECTION("rectangle walls") { config.set_deserialize_strict("wipe_tower_wall_type", "rectangle"); }
    SECTION("rib walls") { config.set_deserialize_strict("wipe_tower_wall_type", "rib"); }
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
    DynamicPrintConfig config = two_material_config(true);
    SECTION("rectangle walls") { config.set_deserialize_strict("wipe_tower_wall_type", "rectangle"); }
    SECTION("rib walls: the box includes the fins") { config.set_deserialize_strict("wipe_tower_wall_type", "rib"); }
    Print print;
    print.set_status_silent();
    print.apply(model, config);
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

TEST_CASE("Two spools of one category purge exactly as they do without isolation", "[PrimeTowerIsolation]")
{
    // Three filaments: 0 = PLA-A (100), 1 = PLA-B (100), 2 = PETG (300), one solid cube each so
    // every layer changes between all three and isolation has nothing to insert. Each toolchange,
    // in particular PLA-A <-> PLA-B, must purge the same amount as with isolation off (the stock
    // tower): sharing a section changes where a purge lands, not its size. The purge is the
    // filament extruded between the LOAD and END markers of the toolchange; what the isolated
    // tower appends after END (the section's own wall and fill) is expected to differ.
    DynamicPrintConfig off = two_material_config(false);
    off.set_num_extruders(3);
    off.set_num_filaments(3);
    off.option<ConfigOptionFloats>("filament_diameter")->values = {1.75, 1.75, 1.75};
    off.option<ConfigOptionStrings>("filament_colour")->values = {"#FF0000", "#00FF00", "#0000FF"};
    off.option<ConfigOptionInts>("filament_adhesiveness_category")->values = {100, 100, 300};
    DynamicPrintConfig on = off;
    on.set("prime_tower_isolate_materials", true);

    Model model;
    for (int extruder = 1; extruder <= 3; ++extruder) {
        ModelObject* obj = model.add_object();
        obj->add_volume(make_cube(10., 10., 4.));
        obj->add_instance();
        obj->config.set("extruder", extruder);
        obj->instances.front()->set_offset(Vec3d(40. + 30. * extruder, 50., 0.));
    }

    Print print_on, print_off;
    print_on.set_status_silent();
    print_off.set_status_silent();
    print_on.apply(model, on);
    print_on.process();
    print_off.apply(model, off);
    print_off.process();

    // Sum of the E words (relative extrusion) in the toolchange's G-code between two markers.
    auto extruded_between = [](const std::string& gcode, const char* from, const char* to) {
        const size_t a = gcode.find(from);
        const size_t b = gcode.find(to, a == std::string::npos ? 0 : a);
        REQUIRE(a != std::string::npos);
        REQUIRE(b != std::string::npos);
        double e = 0.;
        std::istringstream in(gcode.substr(a, b - a));
        for (std::string line; std::getline(in, line);) {
            const size_t pos = line.find(" E");
            if (pos != std::string::npos && line.rfind("G1", 0) == 0) {
                const double v = std::atof(line.c_str() + pos + 2);
                if (v > 0.)
                    e += v;
            }
        }
        return e;
    };
    // (old tool, new tool, purged E) per layer, sorted so the order of the sections does not matter.
    using Purge = std::tuple<int, int, double>;
    auto purges_per_layer = [&](const WipeTowerData& data) {
        std::vector<std::vector<Purge>> out;
        for (const auto& layer : data.tool_changes) {
            std::vector<Purge> purges;
            for (const WipeTower::ToolChangeResult& tcr : layer)
                if (tcr.is_tool_change)
                    purges.emplace_back(tcr.initial_tool, tcr.new_tool,
                                        extruded_between(tcr.gcode, "; CP TOOLCHANGE LOAD", "; CP TOOLCHANGE END"));
            std::sort(purges.begin(), purges.end());
            out.push_back(std::move(purges));
        }
        return out;
    };
    const auto a = purges_per_layer(print_on.wipe_tower_data());
    const auto b = purges_per_layer(print_off.wipe_tower_data());
    REQUIRE(a.size() == b.size());
    size_t pla_to_pla = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        CAPTURE(i);
        REQUIRE(a[i].size() == b[i].size());
        REQUIRE(!a[i].empty());
        for (size_t j = 0; j < a[i].size(); ++j) {
            const auto& [old_on, new_on, e_on] = a[i][j];
            const auto& [old_off, new_off, e_off] = b[i][j];
            CAPTURE(j, old_on, new_on, old_off, new_off);
            CHECK(old_on == old_off);
            CHECK(new_on == new_off);
            CHECK_THAT(e_on, WithinRel(e_off, 1e-6));
            CHECK(e_on > 0.);
            if (old_on < 2 && new_on < 2)
                ++pla_to_pla;
        }
    }
    CHECK(pla_to_pla > 0);
    CHECK(print_on.wipe_tower_data().sections.size() == 2);
}

namespace {
// Extrusions of a layer beyond the footprint in y, split by side, with how many of them are not
// the category of the section on that side. Under isolation only a section's own brim, chamfer
// and rib fins may lie beyond the footprint on its side.
struct BeyondFootprint { size_t front = 0, back = 0, front_wrong_category = 0, back_wrong_category = 0; };
static BeyondFootprint beyond_footprint(const Print& print, const WipeTowerData& data, size_t layer)
{
    const float pw = tower_line_width(print);
    const WipeTower2::Section* front = &data.sections.front();
    const WipeTower2::Section* back  = &data.sections.front();
    for (const auto& s : data.sections) {
        if (s.y_start < front->y_start) front = &s;
        if (s.y_end > back->y_end) back = &s;
    }
    BeyondFootprint out;
    for (const auto& tcr : data.tool_changes[layer]) {
        for (const auto& e : tcr.extrusions) {
            if (e.width <= 0.f)
                continue;
            const int category = category_of(print, e.tool);
            if (e.pos.y() < front->y_start - pw - 0.05f) {
                ++out.front;
                if (category != front->category) ++out.front_wrong_category;
            } else if (e.pos.y() > back->y_end + 0.05f) {
                ++out.back;
                if (category != back->category) ++out.back_wrong_category;
            }
        }
    }
    return out;
}
} // namespace

TEST_CASE("Each isolated section prints its own brim in its own material", "[PrimeTowerIsolation]")
{
    // The brim is per section: the whole-tower brim loops are clipped at the midline of the gap
    // between sections, so the front section's brim is printed by its own category and the back
    // section's by its own. The sections are spaced so that the innermost loop of each goes all
    // the way around: in the gap there is one brim line of each category on layer 1.
    Model model;
    add_gap_objects(model);
    Print print;
    print.set_status_silent();
    print.apply(model, two_material_config(true));
    print.process();
    const WipeTowerData& data = print.wipe_tower_data();
    REQUIRE(data.sections.size() == 2);
    REQUIRE(data.tool_changes.size() > 2);
    const BeyondFootprint b = beyond_footprint(print, data, 0);
    CAPTURE(b.front, b.back, b.front_wrong_category, b.back_wrong_category);
    CHECK(b.front > 0);
    CHECK(b.back > 0);
    CHECK(b.front_wrong_category == 0);
    CHECK(b.back_wrong_category == 0);

    // In the gap between the two sections on layer 1: long horizontal lines of both categories,
    // each on its own side of the midline.
    const auto& front = data.sections[0];
    const auto& back  = data.sections[1];
    REQUIRE(front.y_end < back.y_start);
    const float mid = (front.y_end + back.y_start) / 2.f;
    size_t front_lines = 0, back_lines = 0, misplaced = 0;
    const auto layers = segments_per_layer(data);
    for (const WallSegment& seg : layers[0]) {
        if (std::abs(seg.a.y() - seg.b.y()) > 0.01f || std::abs(seg.a.x() - seg.b.x()) < 5.f)
            continue;
        const float y = seg.a.y();
        if (y <= front.y_end + 0.01f || y >= back.y_start - 0.01f)
            continue; // walls and interiors
        const int category = category_of(print, seg.tool);
        if (category == front.category && y < mid) ++front_lines;
        else if (category == back.category && y > mid) ++back_lines;
        else ++misplaced;
    }
    CAPTURE(front.y_end, back.y_start, mid, front_lines, back_lines, misplaced);
    CHECK(front_lines > 0);
    CHECK(back_lines > 0);
    CHECK(misplaced == 0);
}

TEST_CASE("An isolated section's brim is one continuous flow", "[PrimeTowerIsolation]")
{
    // Only the innermost brim loop of a section goes all the way around. On the gap side the
    // outer loops are cut, and a loose end facing the neighbour's loose end is where a nozzle
    // catches. So the innermost loop's gap-side line is extended outwards past the footprint as
    // far as the outermost loop (a bar) and the outer loops are clipped at that bar. The innermost
    // loop, the bars and the outermost loop are one path, and the loops in between are a serpentine
    // chained onto it (each stepping out along the bar), so that the whole brim is laid in one
    // flow with no pickups: every cut loop ends on its own material, and no brim of one section
    // reaches past its bar towards the other.
    Model model;
    add_gap_objects(model);
    Print print;
    print.set_status_silent();
    print.apply(model, two_material_config(true));
    print.process();
    const WipeTowerData& data = print.wipe_tower_data();
    REQUIRE(data.sections.size() == 2);
    const float pw      = tower_line_width(print);
    const float lh      = float(print.config().initial_layer_print_height.value);
    const float spacing = pw - lh * float(1. - M_PI_4); // brim loop spacing, as WipeTower2 computes it
    const int   loops   = int((float(print.config().prime_tower_brim_width.value) + spacing / 2.f) / spacing);
    REQUIRE(loops >= 3);
    const float width = float(print.config().prime_tower_width.value);
    const auto& front = data.sections[0];
    const auto& back  = data.sections[1];
    REQUIRE(front.y_end < back.y_start);
    // Wall centres: the front section's top wall is at its interior end, the back section's bottom
    // wall one line below its interior start; each bar is one brim spacing beyond its wall.
    const float bar_front = front.y_end + spacing;
    const float bar_back  = back.y_start - pw - spacing;
    const float reach     = float(loops - 1) * spacing; // from the innermost loop to the outermost
    const auto layers = segments_per_layer(data);

    struct Side { size_t bar_segments = 0; size_t joined_bars = 0; size_t beyond_bar = 0; size_t pickups = 0; };
    auto inspect = [&](const WipeTower2::Section& section, float bar_y, bool is_front) {
        Side out;
        const auto& segs = layers[0];
        auto on_bar_line = [&](const WallSegment& seg) { return std::abs(seg.a.y() - bar_y) < 0.05f && std::abs(seg.b.y() - bar_y) < 0.05f; };
        for (size_t i = 0; i < segs.size(); ++i) {
            const WallSegment& seg = segs[i];
            if (category_of(print, seg.tool) != section.category)
                continue;
            const float x0 = std::min(seg.a.x(), seg.b.x()), x1 = std::max(seg.a.x(), seg.b.x());
            // The bar: horizontal, on the bar line, lying outside the footprint and reaching the outermost loop.
            if (on_bar_line(seg)) {
                const bool left  = x1 <= 0.05f && x0 <= -reach - spacing + 0.1f;
                const bool right = x0 >= width - 0.05f && x1 >= width + reach + spacing - 0.1f;
                if (left || right) {
                    ++out.bar_segments;
                    // Joined to the outermost loop: the segment extruded just before or after it
                    // continues from the bar's outer end, off the bar line.
                    const Vec2f outer_end = left ? (seg.a.x() < seg.b.x() ? seg.a : seg.b) : (seg.a.x() > seg.b.x() ? seg.a : seg.b);
                    for (size_t j : {i - 1, i + 1}) {
                        if (j >= segs.size() || on_bar_line(segs[j]))
                            continue;
                        if ((segs[j].a - outer_end).norm() < 0.05f || (segs[j].b - outer_end).norm() < 0.05f) { ++out.joined_bars; break; }
                    }
                }
            }
            // Nothing of this section beyond its bar towards the neighbour, on the outside of the footprint.
            for (const Vec2f& p : {seg.a, seg.b})
                if ((p.x() < -0.3f || p.x() > width + 0.3f) && (is_front ? p.y() > bar_y + 0.3f : p.y() < bar_y - 0.3f))
                    ++out.beyond_bar;
        }
        // Pickups into the brim: a path that starts (after a travel) outside the section's walls.
        // One: the innermost loop and everything beyond it are a single flow.
        for (const WipeTower::ToolChangeResult& tcr : data.tool_changes[0])
            for (const WipeTower::Extrusion& e : tcr.extrusions) {
                if (e.width > 0.f || category_of(print, e.tool) != section.category)
                    continue;
                const bool outside = e.pos.x() < -0.1f || e.pos.x() > width + 0.1f ||
                                     e.pos.y() < section.y_start - pw - 0.1f || e.pos.y() > section.y_end + 0.1f;
                if (outside)
                    ++out.pickups;
            }
        return out;
    };
    const Side f = inspect(front, bar_front, true);
    const Side b = inspect(back, bar_back, false);
    CAPTURE(spacing, loops, reach, bar_front, bar_back, f.bar_segments, f.joined_bars, f.beyond_bar, f.pickups,
            b.bar_segments, b.joined_bars, b.beyond_bar, b.pickups);
    CHECK(f.bar_segments >= 2); // left and right
    CHECK(b.bar_segments >= 2);
    CHECK(f.joined_bars == f.bar_segments);
    CHECK(b.joined_bars == b.bar_segments);
    CHECK(f.beyond_bar == 0);
    CHECK(b.beyond_bar == 0);
    CHECK(f.pickups <= 1);
    CHECK(b.pickups <= 1);
}

TEST_CASE("An isolated section's wall is finished before its brim starts", "[PrimeTowerIsolation]")
{
    // The writer starts a path list at the segment nearest the nozzle and wraps round at the end.
    // With the wall and the brim in one list that could print part of the wall, then the brim,
    // then travel back for the rest of the wall. The brim is extruded on its own, so everything
    // inside the footprint (fill and wall) is done before the first brim line.
    Model model;
    add_gap_objects(model);
    Print print;
    print.set_status_silent();
    print.apply(model, two_material_config(true));
    print.process();
    const WipeTowerData& data = print.wipe_tower_data();
    REQUIRE(data.sections.size() == 2);
    const float pw    = tower_line_width(print);
    const float width = float(print.config().prime_tower_width.value);
    const auto layers = segments_per_layer(data);
    for (const auto& section : data.sections) {
        size_t last_inside = 0, first_outside = SIZE_MAX, inside = 0, outside = 0;
        for (size_t i = 0; i < layers[0].size(); ++i) {
            const WallSegment& seg = layers[0][i];
            if (category_of(print, seg.tool) != section.category)
                continue;
            bool in = true;
            for (const Vec2f& p : {seg.a, seg.b})
                if (p.x() < -0.05f || p.x() > width + 0.05f || p.y() < section.y_start - pw - 0.05f || p.y() > section.y_end + 0.05f)
                    in = false;
            if (in) { last_inside = i; ++inside; }
            else { first_outside = std::min(first_outside, i); ++outside; }
        }
        CAPTURE(section.category, inside, outside, last_inside, first_outside);
        REQUIRE(inside > 0);
        REQUIRE(outside > 0);
        CHECK(last_inside < first_outside);
    }
}

TEST_CASE("Isolated section brim chamfer follows the flag, per section", "[PrimeTowerIsolation]")
{
    // Each section's chamfer loops stack on its own brim in its own material, so the chamfer is
    // no longer forced off: with the flag on, the layers above the first carry loops beyond the
    // footprint on each side in that side's category; with it off, nothing is printed beyond the
    // footprint above the first layer.
    Model model;
    add_gap_objects(model);
    DynamicPrintConfig config = two_material_config(true);
    bool chamfer = true;
    SECTION("flag on") { chamfer = true; }
    SECTION("flag off") { chamfer = false; }
    config.set("prime_tower_brim_chamfer", chamfer);
    Print print;
    print.set_status_silent();
    print.apply(model, config);
    print.process();
    const WipeTowerData& data = print.wipe_tower_data();
    REQUIRE(data.sections.size() == 2);
    REQUIRE(data.tool_changes.size() > 4);
    BeyondFootprint upper;
    for (size_t li = 1; li < data.tool_changes.size(); ++li) {
        const BeyondFootprint b = beyond_footprint(print, data, li);
        upper.front += b.front; upper.back += b.back;
        upper.front_wrong_category += b.front_wrong_category; upper.back_wrong_category += b.back_wrong_category;
    }
    CAPTURE(chamfer, upper.front, upper.back, upper.front_wrong_category, upper.back_wrong_category);
    CHECK(upper.front_wrong_category == 0);
    CHECK(upper.back_wrong_category == 0);
    if (chamfer) {
        CHECK(upper.front > 0);
        CHECK(upper.back > 0);
    } else {
        CHECK(upper.front == 0);
        CHECK(upper.back == 0);
    }
}

TEST_CASE("Isolated sections are spaced for two brim lines when the tower has a brim", "[PrimeTowerIsolation]")
{
    // Between two sections there are two wall lines plus, with a brim, one brim line of each
    // section; without a brim, one line of air.
    Model model;
    add_gap_objects(model);
    DynamicPrintConfig config = two_material_config(true);
    float lines = 0.f;
    SECTION("with brim: four line widths") { config.set("prime_tower_brim_width", 3.); lines = 4.f; }
    SECTION("without brim: three line widths") { config.set("prime_tower_brim_width", 0.); lines = 3.f; }
    Print print;
    print.set_status_silent();
    print.apply(model, config);
    print.process();
    const WipeTowerData& data = print.wipe_tower_data();
    REQUIRE(data.sections.size() == 2);
    const float pw = tower_line_width(print);
    CAPTURE(lines, pw, data.sections[0].y_end, data.sections[1].y_start);
    CHECK_THAT(double(data.sections[1].y_start - data.sections[0].y_end), WithinAbs(double(lines * pw), 0.01));
}

TEST_CASE("Stock tower honours the brim chamfer flag", "[PrimeTowerIsolation]")
{
    // prime_tower_brim_chamfer (developer mode) was honoured by the old tower and dropped by the
    // block tower path; isolation forcing it off brings it back. The stock tower keeps its full
    // depth on every layer with two categories, so its own extent is the same above layer 0 and
    // anything beyond that extent plus a line of fillet tolerance is brim or chamfer.
    Model model;
    add_gap_objects(model);
    DynamicPrintConfig config = two_material_config(false);
    config.set_deserialize_strict("wipe_tower_wall_type", "rectangle");
    bool chamfer = true;
    SECTION("flag on: chamfer loops above the first layer") { chamfer = true; }
    SECTION("flag off: brim on the first layer only") { chamfer = false; }
    config.set("prime_tower_brim_chamfer", chamfer);
    Print print;
    print.set_status_silent();
    print.apply(model, config);
    print.process();
    const WipeTowerData& data = print.wipe_tower_data();
    REQUIRE(data.tool_changes.size() > 6);
    // Tower extent: the y span of the top tower layer, which never carries brim or chamfer.
    float y_min = std::numeric_limits<float>::max(), y_max = std::numeric_limits<float>::lowest();
    for (const auto& tcr : data.tool_changes.back())
        for (const auto& e : tcr.extrusions)
            if (e.width > 0.f) { y_min = std::min(y_min, e.pos.y()); y_max = std::max(y_max, e.pos.y()); }
    REQUIRE(y_min < y_max);
    auto outside = [&](float y) { return y < y_min - 0.05f || y > y_max + 0.05f; };
    size_t first_layer_brim = 0, upper_brim = 0;
    for (size_t li = 0; li + 1 < data.tool_changes.size(); ++li)
        for (const auto& tcr : data.tool_changes[li])
            for (const auto& e : tcr.extrusions)
                if (e.width > 0.f && outside(e.pos.y()))
                    (li == 0 ? first_layer_brim : upper_brim)++;
    CAPTURE(chamfer, y_min, y_max, first_layer_brim, upper_brim);
    CHECK(first_layer_brim > 0);
    if (chamfer)
        CHECK(upper_brim > 0);
    else
        CHECK(upper_brim == 0);
}

namespace {
// Three filaments: 0 = PLA-A (100) 4 mm cube, 1 = PLA-B (100) 1 mm cube, 2 = PETG (300) 4 mm
// cube. While PLA-B prints, the PLA section takes two purges per layer; above 1 mm only one.
static DynamicPrintConfig three_filament_config(bool isolate)
{
    DynamicPrintConfig config = two_material_config(isolate);
    config.set_num_extruders(3);
    config.set_num_filaments(3);
    config.option<ConfigOptionFloats>("filament_diameter")->values = {1.75, 1.75, 1.75};
    config.option<ConfigOptionStrings>("filament_colour")->values = {"#FF0000", "#00FF00", "#0000FF"};
    config.option<ConfigOptionInts>("filament_adhesiveness_category")->values = {100, 100, 300};
    config.set("prime_tower_min_section_depth", 1.);
    return config;
}

static void add_three_filament_objects(Model& model)
{
    const double heights[] = {4., 1., 4.};
    for (int extruder = 1; extruder <= 3; ++extruder) {
        ModelObject* obj = model.add_object();
        obj->add_volume(make_cube(10., 10., heights[extruder - 1]));
        obj->add_instance();
        obj->config.set("extruder", extruder);
        obj->instances.front()->set_offset(Vec3d(40. + 30. * extruder, 50., 0.));
    }
}

struct Span { float bottom, top; };

// Wall lines of a category on a layer: horizontal segments touching one of the tower's x edges
// (purge and fill lines are inset by at least a line width; the entry gap can shorten a wall
// line at one end) and not reaching past them (brim loops and rib fins do). horizontal_wall_ys()
// is not used because it drops lines inside the section's largest extent, which is where a
// shrunken top wall lies. Returns the y span of those lines, or nothing if the category has none.
static std::optional<Span> wall_span(const std::vector<WallSegment>& segs, const Print& print, float width, int category)
{
    std::vector<float> ys;
    for (const WallSegment& s : segs) {
        const float x0 = std::min(s.a.x(), s.b.x()), x1 = std::max(s.a.x(), s.b.x());
        if (category_of(print, s.tool) == category && std::abs(s.a.y() - s.b.y()) < 0.01f && x1 - x0 > 5.f &&
            x0 > -0.1f && x1 < width + 0.1f && (x0 < 0.05f || x1 > width - 0.05f))
            ys.push_back(s.a.y());
    }
    if (ys.empty())
        return std::nullopt;
    return Span{*std::min_element(ys.begin(), ys.end()), *std::max_element(ys.begin(), ys.end())};
}

// Tower width as laid out: rib walls square the footprint, so take it from the widest wall line.
static float laid_out_width(const std::vector<std::vector<WallSegment>>& layers)
{
    float w = 0.f;
    for (const auto& segs : layers)
        for (const WallSegment& s : segs)
            if (std::abs(s.a.y() - s.b.y()) < 0.01f)
                w = std::max(w, std::max(s.a.x(), s.b.x()));
    return w;
}
} // namespace

TEST_CASE("Isolated section shrinks once a spool of its category is no longer used", "[PrimeTowerIsolation]")
{
    // The section follows the purge demand like the stock tower does (never growing upwards),
    // floored at the minimum section depth, instead of keeping its largest depth to the top. The
    // section start and the other section stay where they are.
    DynamicPrintConfig config = three_filament_config(true);
    Model model;
    add_three_filament_objects(model);
    Print print;
    print.set_status_silent();
    print.apply(model, config);
    print.process();
    const WipeTowerData& data = print.wipe_tower_data();
    REQUIRE(data.sections.size() == 2);
    const float pw = tower_line_width(print);
    const auto layers = segments_per_layer(data);
    REQUIRE(layers.size() > 8);
    const float width = laid_out_width(layers);
    auto span = [&](size_t li, int category) { return wall_span(layers[li], print, width, category); };

    // Layer 1 (both PLA spools in use) against the last layer with both sections active.
    const auto low_pla  = span(1, 100);
    const auto low_petg = span(1, 300);
    REQUIRE(low_pla.has_value());
    REQUIRE(low_petg.has_value());
    size_t high = 0;
    for (size_t li = layers.size(); li-- > 2;)
        if (span(li, 100).has_value() && span(li, 300).has_value()) { high = li; break; }
    REQUIRE(high > 6);
    const auto high_pla  = span(high, 100);
    const auto high_petg = span(high, 300);
    CAPTURE(high, pw, low_pla->bottom, low_pla->top, high_pla->bottom, high_pla->top,
            low_petg->bottom, low_petg->top, high_petg->bottom, high_petg->top);
    // The PLA section is at least one line shallower once PLA-B is gone, and starts where it did.
    CHECK(high_pla->top < low_pla->top - pw + 0.01f);
    CHECK_THAT(double(high_pla->bottom), WithinAbs(double(low_pla->bottom), 0.01));
    // The PETG section starts where it did and is no deeper than before.
    CHECK_THAT(double(high_petg->bottom), WithinAbs(double(low_petg->bottom), 0.01));
    CHECK(high_petg->top <= low_petg->top + 0.01f);
    // The reported sections describe the footprint: the largest extent of each over all layers.
    float pla_top_max = 0.f, petg_top_max = 0.f;
    for (size_t li = 0; li < layers.size(); ++li) {
        if (auto s = span(li, 100)) pla_top_max = std::max(pla_top_max, s->top);
        if (auto s = span(li, 300)) petg_top_max = std::max(petg_top_max, s->top);
    }
    const WipeTower2::Section& pla  = section_for(data, 100);
    const WipeTower2::Section& petg = section_for(data, 300);
    CHECK_THAT(double(pla.y_start), WithinAbs(double(low_pla->bottom + pw), 0.05));
    CHECK_THAT(double(pla.y_end), WithinAbs(double(pla_top_max), 0.05));
    CHECK_THAT(double(petg.y_start), WithinAbs(double(low_petg->bottom + pw), 0.05));
    CHECK_THAT(double(petg.y_end), WithinAbs(double(petg_top_max), 0.05));
}

TEST_CASE("Rib walls keep isolated sections at full depth", "[PrimeTowerIsolation]")
{
    // Rib fins brace a section at its corners, so with rib walls a section keeps its largest
    // depth on every layer where its category is active (the stock rib tower keeps its full
    // footprint too), and only rectangle walls taper.
    DynamicPrintConfig config = three_filament_config(true);
    config.set_deserialize_strict("wipe_tower_wall_type", "rib");
    config.set("prime_tower_brim_chamfer", false); // the chamfer loops above the wall would read as a higher top
    Model model;
    add_three_filament_objects(model);
    Print print;
    print.set_status_silent();
    print.apply(model, config);
    print.process();
    const WipeTowerData& data = print.wipe_tower_data();
    REQUIRE(data.sections.size() == 2);
    const auto layers = segments_per_layer(data);
    REQUIRE(layers.size() > 8);
    const WipeTower2::Section& pla = section_for(data, 100);
    // With fins the top wall line stops short of the tower's x edges, so wall_span() does not
    // apply; the section's top on a layer is simply the highest long horizontal line of its
    // category (fins are diagonal, and above layer 0 nothing is printed past the footprint).
    size_t checked = 0;
    for (size_t li = 1; li < layers.size(); ++li) {
        float top = std::numeric_limits<float>::lowest();
        for (const WallSegment& seg : layers[li])
            if (category_of(print, seg.tool) == 100 && std::abs(seg.a.y() - seg.b.y()) < 0.01f && std::abs(seg.a.x() - seg.b.x()) > 5.f)
                top = std::max(top, seg.a.y());
        if (top == std::numeric_limits<float>::lowest())
            continue;
        ++checked;
        CAPTURE(li, top, pla.y_start, pla.y_end);
        CHECK_THAT(double(top), WithinAbs(double(pla.y_end), 0.05));
    }
    CHECK(checked > 8);
}

namespace {
// Extrusions of every layer above the first that lie beyond the footprint in y, with the
// chamfer off so that only rib fins (or nothing) remain.
struct FinCount { size_t front = 0, back = 0, front_wrong_category = 0, back_wrong_category = 0, layers_with_fins = 0; };
static FinCount count_fins(const Print& print, const WipeTowerData& data)
{
    FinCount out;
    for (size_t li = 1; li < data.tool_changes.size(); ++li) {
        const BeyondFootprint b = beyond_footprint(print, data, li);
        out.front += b.front; out.back += b.back;
        out.front_wrong_category += b.front_wrong_category; out.back_wrong_category += b.back_wrong_category;
        if (b.front + b.back > 0) ++out.layers_with_fins;
    }
    return out;
}
} // namespace

TEST_CASE("Isolated rib tower gives the front and back sections fins in their own material", "[PrimeTowerIsolation]")
{
    // The rib outline is the footprint rectangle plus a fin at each corner. Under isolation the
    // front section owns the two front corners and the back section the two back corners, and
    // each prints its fins as part of its own wall loop. Nothing else is printed beyond the
    // footprint above the first layer.
    Model model;
    add_gap_objects(model);
    DynamicPrintConfig config = two_material_config(true);
    config.set_deserialize_strict("wipe_tower_wall_type", "rib");
    config.set("prime_tower_brim_chamfer", false); // so that only fins lie beyond the footprint above layer 1
    Print print;
    print.set_status_silent();
    print.apply(model, config);
    print.process();
    const WipeTowerData& data = print.wipe_tower_data();
    REQUIRE(data.sections.size() == 2);
    REQUIRE(data.tool_changes.size() > 4);
    const FinCount fins = count_fins(print, data);
    CAPTURE(fins.front, fins.back, fins.front_wrong_category, fins.back_wrong_category, fins.layers_with_fins, data.tool_changes.size());
    CHECK(fins.front > 0);
    CHECK(fins.back > 0);
    CHECK(fins.front_wrong_category == 0);
    CHECK(fins.back_wrong_category == 0);
    // Fins shrink with height and vanish at the top, so expect them on at least the lower half.
    CHECK(fins.layers_with_fins >= (data.tool_changes.size() - 1) / 2);
}

TEST_CASE("Cone walls fall back to rectangle walls under isolation with a warning", "[PrimeTowerIsolation]")
{
    // The cone is a skirt printed on every layer by that layer's tool, so it would stack
    // categories like the shared wall did. There is no per-section equivalent, so it is not
    // printed; validate() says so.
    Model model;
    add_gap_objects(model);
    DynamicPrintConfig config = two_material_config(true);
    config.set_deserialize_strict("wipe_tower_wall_type", "cone");
    config.set("prime_tower_brim_chamfer", false); // so that nothing but a cone could lie beyond the footprint above layer 1
    Print print;
    print.set_status_silent();
    print.apply(model, config);
    StringObjectException warning;
    const StringObjectException err = print.validate(&warning);
    CAPTURE(err.string, warning.string);
    CHECK(err.string.empty());
    CHECK(warning.string.find("one") != std::string::npos); // "Cone"/"cone"
    CHECK(warning.opt_key == "wipe_tower_wall_type");
    print.process();
    const WipeTowerData& data = print.wipe_tower_data();
    REQUIRE(data.sections.size() == 2);
    const FinCount fins = count_fins(print, data);
    CAPTURE(fins.front, fins.back);
    CHECK(fins.front == 0);
    CHECK(fins.back == 0);
}
