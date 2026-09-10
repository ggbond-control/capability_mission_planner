#include <capability_mission_planner/live_reservation.hpp>

#include <iostream>
#include <stdexcept>

using capability_mission_planner::offline::LiveReservationTable;
using capability_mission_planner::offline::ResourceReservation;

int main()
{
    try
    {
        LiveReservationTable table;
        if (!table.try_reserve({"door", "a", 2, 5}) ||
            table.try_reserve({"door", "b", 5, 7}) ||
            !table.try_reserve({"door", "b", 6, 7}))
            throw std::runtime_error("reservation overlap rules failed");
        table.update_progress("a", 6);
        if (table.reservations().size() != 1U || table.reservations().front().robot_id != "b")
            throw std::runtime_error("progress did not release an expired reservation");
        table.release_robot("b");
        if (!table.reservations().empty())
            throw std::runtime_error("release failed");
        if (table.try_reserve(ResourceReservation{"", "c", 0, 1}) ||
            table.try_reserve(ResourceReservation{"door", "c", 3, 2}))
            throw std::runtime_error("invalid reservation accepted");
        std::cout << "live reservation tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "live reservation tests failed: " << error.what() << '\n';
        return 1;
    }
}
