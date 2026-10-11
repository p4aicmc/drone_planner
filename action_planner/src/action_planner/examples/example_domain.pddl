;; Minimal domain used by the framework's standalone example.
;; Every durative action here is executed by a node of the same name that
;; serves /action/<name> (see launch/example_launch.py).
(define (domain action_planner_example)

    (:requirements :typing :strips :fluents :durative-actions)

    (:types
        room - object
    )

    (:predicates
        (at ?r - room)
        (inspected ?r - room)
    )

    (:functions
        (distance ?from - room ?to - room)
        (total_distance)
    )

    (:durative-action move
        :parameters (?from - room ?to - room)
        :duration (= ?duration 2)
        :condition (and
            (at start (at ?from))
        )
        :effect (and
            (at start (not (at ?from)))
            (at end (at ?to))
            (at end (increase (total_distance) (distance ?from ?to)))
        )
    )

    (:durative-action inspect
        :parameters (?r - room)
        :duration (= ?duration 1)
        :condition (and
            (over all (at ?r))
        )
        :effect (and
            (at end (inspected ?r))
        )
    )
)
