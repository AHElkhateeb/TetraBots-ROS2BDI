(define (domain transporting-agent)
    (:requirements :strips :typing :fluents :durative-actions)

    (:types
        waypoint
        robot
        payload
        tool
    )

    (:predicates
        (in ?r - robot ?wp - waypoint)
        (payload_in ?p - payload ?wp - waypoint)
        (transport_alone ?r - robot)
        (transport_cooperatively ?r ?r1 ?r2- robot)
        (tool_mounted ?r - robot ?t - tool)
        (tool_required ?p - payload ?t - tool)
        (holding ?r - robot ?p - payload)
        (recharging_station ?wp - waypoint)
        (tool_changing_station ?wp - waypoint)
        (free ?r - robot)
        (workfree ?r - robot)
        (fully_recharged ?r - robot)
    )

    (:functions
        (battery_charge ?r - robot)
    )

    (:durative-action moveto
        :parameters (?r - robot ?wp_from ?wp_to - waypoint)
        :duration (= ?duration 15)
        :condition (and
            (at start (in ?r ?wp_from))
            (at start (workfree ?r))
            (over all (transport_alone ?r))
            (over all (> (battery_charge ?r) 10))
        )
        :effect (and
            (at start (not(workfree ?r)))
            (at start (not(in ?r ?wp_from)))
            (at end (in ?r ?wp_to))
            (at end (workfree ?r))
            (at end (decrease (battery_charge ?r) 5))
        )
    )

    (:durative-action charge
        :parameters (?r - robot ?wp - waypoint)
        :duration (= ?duration 30)
        :condition (and
            (at start (workfree ?r))
            (over all (in ?r ?wp))
            (over all (recharging_station ?wp))
        )
        :effect (and
            (at start (not(workfree ?r)))
            (at end (workfree ?r))
            (at end (fully_recharged ?r))
            (at end (assign (battery_charge ?r) 100))
        )
    )

    (:durative-action pickup
        :parameters (?r - robot ?wp - waypoint ?p - payload ?t - tool)
        :duration (= ?duration 5)
        :condition (and
            (at start (workfree ?r))
            (at start (payload_in ?p ?wp))
            (at start (free ?r))
            (over all (transport_alone ?r))
            (over all (tool_required ?p ?t))
            (over all (tool_mounted ?r ?t))
            (over all (in ?r ?wp))
            (over all (> (battery_charge ?r) 10))
        )
        :effect (and
            (at start (not(workfree ?r)))
            (at start (not(payload_in ?p ?wp)))
            (at start (not(free ?r)))
            (at end (workfree ?r))
            (at end (holding ?r ?p))
        )
    )

    (:durative-action drop
        :parameters (?r - robot ?wp - waypoint ?p - payload)
        :duration (= ?duration 5)
        :condition (and
            (at start (workfree ?r))
            (at start (holding ?r ?p))
            (over all (transport_alone ?r))
            (over all (in ?r ?wp))
            (over all (> (battery_charge ?r) 10))
        )
        :effect (and
            (at start (not(workfree ?r)))
            (at start (not(holding ?r ?p)))
            (at end (free ?r))
            (at end (workfree ?r))
            (at end (payload_in ?p ?wp))
        )
    )

    (:durative-action change_tool
        :parameters (?r - robot ?wp - waypoint ?t1 - tool ?t2 - tool)
        :duration (= ?duration 10)
        :condition (and
            (at start (workfree ?r))
            (at start (tool_mounted ?r ?t1))
            (over all (in ?r ?wp))
            (over all (tool_changing_station ?wp))
            (over all (> (battery_charge ?r) 10))
        )
        :effect (and
            (at start (not(workfree ?r)))
            (at start (not(tool_mounted ?r ?t1)))
            (at end (workfree ?r))
            (at end (tool_mounted ?r ?t2))
        )
    )

    (:durative-action pickup_cooperatively
        :parameters (?r ?r1 ?r2 - robot ?wp - waypoint ?p - payload ?t - tool)
        :duration (= ?duration 5)
        :condition (and
            (at start (workfree ?r))
            (at start (payload_in ?p ?wp))
            (at start (free ?r))
            (over all (transport_cooperatively ?r ?r1 ?r2))
            (over all (tool_required ?p ?t))
            (over all (tool_mounted ?r ?t))
            (over all (in ?r ?wp))
            (over all (> (battery_charge ?r) 10))
        )
        :effect (and
            (at start (not(workfree ?r)))
            (at start (not(payload_in ?p ?wp)))
            (at start (not(free ?r)))
            (at end (workfree ?r))
            (at end (holding ?r ?p))
        )
    )

    (:durative-action moveto_cooperatively
        :parameters (?r ?r1 ?r2 - robot ?wp_from ?wp_to - waypoint)
        :duration (= ?duration 15)
        :condition (and
            (at start (in ?r ?wp_from))
            (at start (workfree ?r))
            (over all (transport_cooperatively ?r ?r1 ?r2))
            (over all (> (battery_charge ?r) 10))
        )
        :effect (and
            (at start (not(workfree ?r)))
            (at start (not(in ?r ?wp_from)))
            (at end (in ?r ?wp_to))
            (at end (workfree ?r))
            (at end (decrease (battery_charge ?r) 5))
        )
    )

    (:durative-action drop_cooperatively
        :parameters (?r ?r1 ?r2 - robot ?wp - waypoint ?p - payload)
        :duration (= ?duration 5)
        :condition (and
            (at start (workfree ?r))
            (at start (holding ?r ?p))
            (over all (transport_cooperatively ?r ?r1 ?r2))
            (over all (in ?r ?wp))
            (over all (> (battery_charge ?r) 10))
        )
        :effect (and
            (at start (not(workfree ?r)))
            (at start (not(holding ?r ?p)))
            (at end (free ?r))
            (at end (workfree ?r))
            (at end (not(transport_cooperatively ?r ?r1 ?r2)))
            (at end (transport_alone ?r))
            (at end (payload_in ?p ?wp))
        )
    )

)