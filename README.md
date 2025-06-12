# Highway Path Planner

This project is a C implementation for a competitive programming problem from the "Algorithms and Data Structures" course (2022-2023). It involves managing a series of service stations on a highway and planning optimal routes for electric vehicles.

The solution uses two layers of Red-Black Trees for efficient data management and a custom memory allocator to minimize overhead from `malloc` calls.

## Problem Description

A highway is described as a sequence of service stations, each uniquely identified by its distance (a non-negative integer) from the start of the highway.

-   **Stations and Cars**: Each station has a fleet of electric rental cars. Each car is defined by its autonomy (range) in kilometers. A station can have up to 512 vehicles.
-   **Travel**: A driver can rent a car at a station `s` and travel to any station `t` provided that the distance between them is less than or equal to the car's autonomy. The journey is always one-way (distance from the start must increase). A new car is rented at every stop.
-   **Path Planning**: The goal is to find the best path between a starting station and a destination station. The criteria for "best" are:
    1.  **Minimum Stops**: The path must have the fewest possible intermediate stops.
    2.  **Lexicographically Smallest Path**: If multiple paths have the same minimum number of stops, the chosen path must be the one that is "lexicographically smallest." This means at the first point of divergence from another optimal path, our path must choose the station with the smaller distance from the highway's start.

The system must support adding/removing stations, adding/scrapping cars, and planning paths.


## Data Structures

-   **Stations**: A Red-Black Tree stores the stations, keyed by their `distance`. This allows for O(log N) search, insertion, and deletion. The nodes are also linked in a doubly-linked list to allow for efficient linear traversal by distance.
-   **Cars**: Each station contains another Red-Black Tree to store its fleet of cars, keyed by `autonomy`. This allows finding the car with maximum autonomy (a cached pointer to the rightmost node) in O(1) time after the initial O(log M) search.
-   **Memory Management**: A custom "arena" allocator pre-allocates large blocks of memory for all nodes to avoid the performance cost of repeated `malloc` calls during runtime.
