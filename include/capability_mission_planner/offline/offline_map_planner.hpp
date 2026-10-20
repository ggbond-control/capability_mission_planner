#pragma once

#include <capability_mission_planner/core/mission_planner.hpp>

#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace capability_mission_planner::offline
{
    struct GridPosition
    {
        std::string map_id;
        int x = 0;
        int y = 0;

        bool operator==(const GridPosition &other) const;
        bool operator!=(const GridPosition &other) const { return !(*this == other); }
        bool operator<(const GridPosition &other) const;
    };

    struct MetricPose
    {
        std::string map_id;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double yaw = 0.0;
    };

    enum class CoordinateRepresentation
    {
        Grid,
        LocalXY,
        RootXY
    };

    struct MapLayer
    {
        std::string id;
        std::filesystem::path yaml_path;
        std::filesystem::path image_path;
        int width = 0;
        int height = 0;
        double resolution = 0.0;
        double origin_x = 0.0;
        double origin_y = 0.0;
        double origin_yaw = 0.0;
        double root_x = 0.0;
        double root_y = 0.0;
        double root_yaw = 0.0;
        std::vector<unsigned char> traversable;
        std::vector<float> clearance_m;
        std::vector<unsigned char> inflated_cost;
        bool loaded_from_persistent_cache = false;
        double cache_validation_seconds = 0.0;
        double cache_read_seconds = 0.0;
        double preprocess_seconds = 0.0;

        bool is_traversable(int x, int y) const;
        double clearance(int x, int y) const;
        unsigned char cost(int x, int y) const;
        GridPosition local_to_grid(double x, double y) const;
        MetricPose grid_to_local(const GridPosition &position) const;
        MetricPose local_to_root(const MetricPose &pose) const;
        MetricPose root_to_local(const MetricPose &pose) const;
    };

    struct MapTransition
    {
        std::string id;
        std::string from_map;
        std::string to_map;
        MetricPose root_pose;
        bool bidirectional = false;
        std::string type;
        GridPosition from_cell;
        GridPosition to_cell;
    };

    struct MapLoadOptions
    {
        bool allow_unknown = false;
        double inflation_radius = 0.0;
        double inscribed_radius = 0.0;
        double cost_scaling_factor = 10.0;
        bool persistent_cache = true;
        std::filesystem::path cache_directory;
        std::optional<double> free_thresh_override;
        std::optional<double> occupied_thresh_override;
    };

    struct MultiMapBundle
    {
        std::filesystem::path directory;
        std::map<std::string, MapLayer> maps;
        std::vector<MapTransition> transitions;
        std::size_t map_cache_hits = 0;
        std::size_t map_cache_misses = 0;
        double map_cache_validation_seconds = 0.0;
        double map_cache_read_seconds = 0.0;
        double map_preprocess_seconds = 0.0;

        const MapLayer &map(const std::string &id) const;
        bool traversable(const GridPosition &position) const;
        bool is_multi_map() const { return maps.size() > 1U; }
    };

    class MapBundleLoader
    {
    public:
        static std::shared_ptr<const MultiMapBundle> load(const std::filesystem::path &directory, const MapLoadOptions &options = {});
    };

    struct SharedResource
    {
        std::string id;
        std::vector<GridPosition> cells;
        std::size_t capacity = 1;
        double buffer_seconds = 0.0;
    };

    struct TraversalOptions
    {
        double time_step_seconds = 0.1;
        double nominal_speed_mps = 0.5;
        double default_transition_seconds = 5.0;
        double map_switch_seconds = 2.0;
        double obstacle_cost_weight = 1.0;
        bool allow_diagonal = true;
        bool downsample_costmap = false;
        unsigned int coarse_search_factor = 1;
        std::map<std::string, double> transition_seconds{{"stairs", 8.0}, {"elevator", 15.0}};
        std::map<std::string, CapabilitySet> transition_requirements{{"stairs", {"stairs"}}};
        double resource_buffer_seconds = 0.0;
        std::size_t coordination_max_high_level_nodes = 128U;
        bool coordination_local_repair = true;
        double coordination_repair_budget_seconds = 0.2;
        std::size_t coordination_repair_max_expanded_nodes = 20000U;
        std::vector<SharedResource> shared_resources;
    };

    struct PathStep
    {
        GridPosition position;
        int arrival_tick = 0;
        std::string transition_id;
    };

    struct MultiMapPath
    {
        std::vector<PathStep> steps;
        int travel_ticks = 0;
    };

    struct PathQueryProfile
    {
        double required_clearance_m = 0.0;
        double nominal_speed_mps = 0.0;
    };

    struct PathPlannerStats
    {
        std::size_t estimate_requests = 0;
        std::size_t plan_requests = 0;
        std::size_t cache_hits = 0;
        std::size_t grid_searches = 0;
        std::size_t a_star_searches = 0;
        std::size_t expanded_nodes = 0;
        double grid_search_seconds = 0.0;
        std::size_t distance_field_searches = 0;
        double distance_field_seconds = 0.0;
    };

    struct CoordinationStats
    {
        bool prioritized_attempted = false;
        bool prioritized_succeeded = false;
        bool cbs_started = false;
        bool cbs_succeeded = false;
        std::size_t prioritized_low_level_searches = 0;
        std::size_t prioritized_low_level_expanded_nodes = 0;
        std::size_t cbs_high_level_expanded_nodes = 0;
        std::size_t cbs_low_level_searches = 0;
        std::size_t cbs_low_level_expanded_nodes = 0;
        std::size_t conflict_checks = 0;
        double conflict_check_seconds = 0.0;
        std::size_t total_wait_ticks = 0;
        std::size_t reservation_intervals = 0;
        std::size_t reservation_queries = 0;
        std::size_t safe_interval_expanded_nodes = 0;
        bool local_repair_attempted = false;
        bool local_repair_accepted = false;
        bool local_repair_budget_exhausted = false;
        double local_repair_seconds = 0.0;
        std::size_t local_repair_expanded_nodes = 0;
        int local_repair_makespan_before_ticks = 0;
        int local_repair_makespan_after_ticks = 0;
        std::vector<std::size_t> local_repair_robots;
        // Rejected candidate departures, grouped by affected robot / blocker.
        // These are search diagnostics, not executed wait durations.
        std::vector<std::vector<std::size_t>> blocking_departure_ticks_by_robot;
        std::vector<std::size_t> prioritized_expanded_nodes_by_robot;
        std::vector<std::size_t> prioritized_searches_by_robot;
        std::vector<std::size_t> route_frames_by_robot;
        std::vector<std::vector<std::size_t>> prioritized_frame_expansions_by_robot;
    };

    class MultiMapPathPlanner
    {
    public:
        MultiMapPathPlanner(
            std::shared_ptr<const MultiMapBundle> bundle,
            TraversalOptions options = {});

        MultiMapPath plan(
            const GridPosition &start,
            const GridPosition &goal,
            const CapabilitySet &capabilities = {}) const;
        MultiMapPath plan(
            const GridPosition &start,
            const GridPosition &goal,
            const CapabilitySet &capabilities,
            double required_clearance_m) const;
        MultiMapPath plan(
            const GridPosition &start,
            const GridPosition &goal,
            const CapabilitySet &capabilities,
            double required_clearance_m,
            double nominal_speed_mps) const;
        MultiMapPath plan_exact(
            const GridPosition &start,
            const GridPosition &goal,
            const CapabilitySet &capabilities = {},
            double required_clearance_m = 0.0,
            double nominal_speed_mps = 0.0) const;
        int distance(
            const GridPosition &start,
            const GridPosition &goal,
            const CapabilitySet &capabilities = {}) const;
        int distance(
            const GridPosition &start,
            const GridPosition &goal,
            const CapabilitySet &capabilities,
            double required_clearance_m) const;
        int estimate_distance(
            const GridPosition &start,
            const GridPosition &goal,
            const CapabilitySet &capabilities,
            double required_clearance_m,
            double nominal_speed_mps = 0.0) const;
        void precompute_estimates(
            const std::vector<GridPosition> &positions,
            const std::vector<PathQueryProfile> &profiles) const;
        const MultiMapBundle &bundle() const { return *_bundle; }
        const TraversalOptions &options() const { return _options; }
        PathPlannerStats stats() const;
        void reset_stats() const;

    private:
        std::shared_ptr<const MultiMapBundle> _bundle;
        TraversalOptions _options;
        mutable std::map<std::string, MultiMapPath> _cache;
        mutable std::map<std::string, MultiMapPath> _segment_cache;
        mutable std::map<std::string, int> _estimate_cache;
        mutable std::shared_ptr<const MultiMapBundle> _coarse_bundle;
        mutable std::shared_ptr<MultiMapPathPlanner> _coarse_planner;
        mutable PathPlannerStats _stats;

        MultiMapPath plan_impl(
            const GridPosition &start,
            const GridPosition &goal,
            const CapabilitySet &capabilities,
            double required_clearance_m,
            double nominal_speed_mps,
            bool force_full_resolution) const;
    };

    struct MappedRobot
    {
        std::string id;
        GridPosition start;
        CapabilitySet capabilities;
        std::optional<GridPosition> return_position;
        double clearance_radius_m = 0.0;
        double safety_margin_m = 0.0;
        double nominal_speed_mps = 0.0;
        double footprint_radius_m = 0.0;
        CoordinateRepresentation coordinate_representation = CoordinateRepresentation::Grid;
    };

    struct MappedTask
    {
        ConstTaskBookingPtr booking;
        TaskHeader header;
        GridPosition location;
        CapabilitySet requirements;
        double position_tolerance_m = 0.0;
        CoordinateRepresentation coordinate_representation = CoordinateRepresentation::Grid;

        const std::string &id() const { return booking->id(); }
        int service_duration_seconds() const;
        bool high_priority() const;
    };

    MappedTask make_mapped_task(
        std::string id,
        GridPosition location,
        CapabilitySet requirements,
        std::string category,
        int service_seconds,
        bool high_priority = false,
        int earliest_start_seconds = 0);

    struct MappedRouteStop
    {
        GridPosition location;
        std::vector<std::size_t> task_indices;
        int service_ticks = 0;
        double position_tolerance_m = 0.0;
    };

    struct MappedRobotRoute
    {
        std::size_t robot_index = 0;
        std::vector<MappedRouteStop> stops;
        std::vector<MultiMapPath> segments;
        int travel_ticks = 0;
        int service_ticks = 0;
        int load_ticks() const { return travel_ticks + service_ticks; }
    };

    struct TimedMapState
    {
        GridPosition position;
        int tick = 0;
        std::string transition_id;
        std::size_t route_frame = 0;
        std::string resource;
    };

    enum class NavigationCheckpointType
    {
        Start,
        Task,
        Turn,
        ResourceEntry,
        ResourceExit,
        TransitionEntry,
        TransitionExit,
        Holding,
        Finish
    };

    struct NavigationCheckpoint
    {
        NavigationCheckpointType type = NavigationCheckpointType::Turn;
        GridPosition position;
        int arrival_tick = 0;
        int departure_tick = 0;
        std::string resource;
        std::string transition_id;
        std::string task_id;
        std::string id;
    };

    struct TrafficEvent
    {
        std::string type;
        int start_tick = 0;
        int end_tick = 0;
        GridPosition position;
        std::string resource;
        std::string reason;
        std::string checkpoint_id;
    };

    struct AllocationOptions
    {
        std::size_t max_passes = 8U;
        std::size_t max_candidate_evaluations = 50000U;
        double budget_seconds = 1.0;
        bool balance_task_counts = true;
        // Count balancing may increase both maximum and total estimated load by
        // at most this fraction relative to the time-optimized allocation.
        double balance_load_slack = 0.10;
    };

    struct AllocationStats
    {
        std::size_t candidate_evaluations = 0;
        std::size_t relocation_moves = 0;
        std::size_t balancing_moves = 0;
        bool budget_exhausted = false;
        bool balance_reverted = false;
        double local_search_seconds = 0.0;
        std::vector<std::size_t> initial_task_counts;
        std::vector<std::size_t> final_task_counts;
    };

    struct OfflineMissionPlan
    {
        std::vector<MappedRobotRoute> routes;
        std::vector<std::vector<TimedMapState>> schedules;
        std::vector<std::vector<NavigationCheckpoint>> navigation_checkpoints;
        std::vector<std::vector<TrafficEvent>> traffic_events;
        std::vector<SharedResource> shared_resources;
        int maximum_load_ticks = 0;
        int total_load_ticks = 0;
        double time_step_seconds = 0.1;
        PathPlannerStats allocation_path_stats;
        PathPlannerStats total_path_stats;
        double allocation_seconds = 0.0;
        double estimate_precompute_seconds = 0.0;
        double final_path_seconds = 0.0;
        double coordination_seconds = 0.0;
        CoordinationStats coordination_stats;
        AllocationStats allocation_stats;
    };

    class OfflineMissionPlanner
    {
    public:
        OfflineMissionPlanner(
            MultiMapPathPlanner path_planner,
            ObjectiveWeights weights = {},
            AllocationOptions allocation = {});

        OfflineMissionPlan plan(
            const std::vector<MappedRobot> &robots,
            const std::vector<MappedTask> &tasks,
            bool coordinate_conflicts = true) const;

    private:
        MultiMapPathPlanner _path_planner;
        ObjectiveWeights _weights;
        AllocationOptions _allocation;
    };

    struct ExportOptions
    {
        int path_thickness = 3;
        bool draw_grid = false;
        bool filter_navigation_checkpoint_types = false;
        std::set<std::string> navigation_checkpoint_types;
        // Optional, synchronized replay of the exact coordination schedules.
        bool animation = false;
        int animation_fps = 10;
        double animation_speedup = 20.0;
        int animation_max_frames = 600;
        int animation_width = 1280;
    };

    class PlanExporter
    {
    public:
        static std::string to_json(
            const MultiMapBundle &bundle,
            const std::vector<MappedRobot> &robots,
            const std::vector<MappedTask> &tasks,
            const OfflineMissionPlan &plan,
            const ExportOptions &options = {});
        static void write(
            const std::filesystem::path &output_directory,
            const MultiMapBundle &bundle,
            const std::vector<MappedRobot> &robots,
            const std::vector<MappedTask> &tasks,
            const OfflineMissionPlan &plan,
            const ExportOptions &options = {});
    };
} // namespace capability_mission_planner::offline
