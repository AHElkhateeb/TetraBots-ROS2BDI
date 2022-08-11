(define (domain organizing-agent)
    (:requirements :strips :typing :fluents :durative-actions)

    (:types
        waypoint
        payload
        tool

        number
    )

    (:constants
        one - number
        two - number
        three - number    
    )

    (:predicates
        (payload_in ?p - payload ?wp - waypoint)
        (payload_should_be_in ?p - payload ?wp - waypoint ?t - tool ?num - number)
    )

    (:durative-action ask_for_transportation
        :parameters (?p - payload ?wp_from ?wp_to - waypoint ?t - tool ?num - number)
        :duration (= ?duration 150)
        :condition (and
            (at start (payload_in ?p ?wp_from))
            (over all (payload_should_be_in ?p ?wp_to ?t ?num))
        )
        :effect (and
            (at start (not(payload_in ?p ?wp_from)))

            (at end (not(payload_should_be_in ?p ?wp_to ?t ?num)))

            (at end (payload_in ?p ?wp_to))
        )
    )
)
