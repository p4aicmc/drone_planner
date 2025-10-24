(define (domain harpia)

    (:requirements  :typing  :strips  :disjunctive-preconditions  :equality :fluents :durative-actions)

    (:types
        region - object
        base - region
    )

    (:predicates
        (at ?region - region)
        (been_at ?region - region)
        (taken_image ?region - region)
        (pulverized ?region - region)
        (pulverize_goal ?region - region)
        (picture_goal ?region - region)

    )

   
    (:functions
        (battery_amount)                                     ;; Variavel q controla bateria em porcentagem
        (input_amount)                                       ;; quantidade de insumo
        (battery_capacity)                                   ;;capacidade maxima bateria
        (input_capacity)                                     ;;capacidade maxima de insumo
        (distance ?from_region - region ?to_region - region) ;;distancia entre regioes em metros
        (mission_length)
    )


    (:durative-action go_to
        :parameters (
             ?from_region - region 
             ?to_region - region)
        :duration
            (= ?duration 5)
        :condition (and
            (at start (at ?from_region))
            ; (at end (> (battery_amount) 0))
            (over all (> (battery_capacity) 0))
            ; (at start (> (battery_amount) 0))
        )
        :effect (and 
            (at start (not (at ?from_region)))
            (at end (been_at ?to_region))
            (at end (at ?to_region))
            ; (at end (decrease (battery_amount) 1))
            (at end(increase (mission_length) (/ (distance ?from_region ?to_region) 2)))
        )
    )
    
    (:durative-action take_image
        :parameters (
            ?region - region
        )
        :duration
            (= ?duration 1)
        :condition(and
            (over all(at ?region))
            (over all(picture_goal ?region))
            (at start (> (battery_amount) 50))
       )
        :effect(and
            (at end(taken_image ?region))
            (at end(increase (mission_length) 1000))
            ; (at end(decrease (battery_amount) 10))
        )
    )
    (:durative-action pulverize_region
        :parameters (
            ?region - region)
        :duration
            (= ?duration 1)
        :condition(and
            (over all(at ?region))
            (over all(pulverize_goal ?region))
            (at start(> (input_amount) 0))
       )
        :effect(and
            (at end(pulverized ?region))
            (at end(increase (mission_length) 314))
            (at end(decrease (input_amount) 1))
            ; (at end(decrease (battery_amount) 10 ))
        )
    )
    (:durative-action recharge_battery
        :parameters (?base - base)
        :duration
            (= ?duration 1)
        :condition (and
            (over all(at ?base))
        )
        :effect 
        (and
            (at end (assign (battery_amount) (battery_capacity)))
        )
    )

    (:durative-action recharge_input
        :parameters (?base - base)
        :duration
            (= ?duration 1)
        :condition (and
            (over all(at ?base))
            (at start(< (input_amount) (/ (input_capacity) 2)))
        )
        :effect 
        (and
            (at end(assign (input_amount) (input_capacity)))
        )
    )

    ; (:durative-action test
    ;     :parameters (?reg1 - region
    ;                  ?reg2 - region) 
    ;     :duration
    ;         (= ?duration 1)
    ;     :condition (and
    ;         (at start (> (battery_amount) (/ (battery_capacity) 2)))
    ;     )
    ;     :effect 
    ;     (and
    ;         (at end(assign (input_amount) (input_capacity)))
    ;     )
    ; )


)