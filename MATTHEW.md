The following is a rough list of the components I have for long-term planning:
1. Route Ingest: Matches the route to the lane graph at the start and continuously re-matches as the car goes.
2. Progress Tracker: Measures progress and deviation and detects a missed turn and catches localization drift.
3. Lane-Level Planner: Plans every lane for the current window.
4. Upcoming Scanner: Looks up to a couple hundred meters ahead to list upcoming events along the planned path.
5. Re-Planner: Gets around obstacles and recovers from a missed turn with a plan to rejoin the given route.
6. Route Intent Composer: Publishes the plan of the current window for other modules to read.
7. Window Manager: Creates a detailed lane-level plan for the next window, moving it forward as the car progresses.
8. Health Monitor: Tracks budgets across the whole route like time taken and number of infractions and stalls