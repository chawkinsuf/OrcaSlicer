#include <catch2/catch_test_macros.hpp>
#include "test_utils.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/GCode/ToolOrdering.hpp"

#include <algorithm>
#include <vector>

using namespace Slic3r;

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
    CHECK(config.opt_float("prime_tower_min_section_depth") == 10.);
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
    for (const WipeTowerData::Section& s : data.sections) {
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
