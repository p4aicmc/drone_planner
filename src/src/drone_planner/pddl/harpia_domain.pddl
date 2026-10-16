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
        (is_flying)
        (is_not_flying)
    )

   
    (:functions
        (battery_amount)                                     ;; Variavel q controla bateria em porcentagem
        (battery_capacity)                                   ;;capacidade maxima bateria
        (battery_recharge_rate)
        (distance ?from_region - region ?to_region - region) ;;distancia entre regioes em metros
        (mission_length)
        (cruising_speed)
        (take_off_time)
        (land_time)
        (take_image_time ?region - region)
    )


    (:durative-action go_to
        :parameters (
             ?from_region - region 
             ?to_region - region)
        :duration
            (= ?duration ( /(distance ?from_region ?to_region) (cruising_speed)))
        :condition (and
            (at start (not (= ?from_region ?to_region)))
            (at start (at ?from_region))
            (over all (> (battery_capacity) 0))
            (over all (is_flying))
        )
        :effect (and 
            (at start (not (at ?from_region)))
            (at end (been_at ?to_region))
            (at end (at ?to_region))
            ; (at end (decrease (battery_amount) 1))
            (at end (increase (mission_length) (distance ?from_region ?to_region)))
        )
    )
    
    (:durative-action take_image
        :parameters (
            ?region - region
        )
        :duration
            (= ?duration (take_image_time ?region))
        :condition(and
            (over all(at ?region))
            (over all(picture_goal ?region))
            (over all (is_flying))
            (at start (> (battery_amount) (* 0.4 (battery_capacity))))
       )
        :effect(and
            (at end(taken_image ?region))
            ; (at end(decrease (battery_amount) 10))
        )
    )

    (:durative-action recharge_battery
        :parameters (?base - base)
        :duration
            (= ?duration 
                (/
                    (- (battery_capacity) (battery_amount))
                    (battery_recharge_rate)
                )
            )
        :condition (and
            (over all(at ?base))
            (over all (is_not_flying))
        )
        :effect 
        (and
            (at end (assign (battery_amount) (battery_capacity)))
        )
    )

    (:durative-action take_off
        :parameters ()
        :duration (= ?duration (take_off_time))
        :condition (and 
            (at start (is_not_flying))
        )
        :effect (and 
            (at end (and 
            (is_flying)
            (not (is_not_flying))
            ))
        )
    )

    (:durative-action land
        :parameters ()
        :duration (= ?duration (land_time))
        :condition (and 
            (at start (is_flying))
        )
        :effect (and 
            (at end (and 
            (not (is_flying))
            (is_not_flying)
            ))
        )
    )
)
