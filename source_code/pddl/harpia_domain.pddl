(define (domain harpia)

    (:requirements  :typing  :strips  :disjunctive-preconditions  :equality :fluents :durative-actions)

    (:types
        region - object
        base - region
    )

    (:predicates

        (at ?region - region)
        (been_at ?region - region)
        (carry)  
        
        (can_spray)
        ;;se pode carregar/descarregar
        (can_recharge)
        ;se já tirou a foto
        (taken_image ?region - region)
        ;se pulverizou
        (pulverized ?region - region)
        ; (canGo)
        (can_take_pic)
        (its_not_base ?region - region)
        (pulverize_goal ?region - region)
        (picture_goal ?region - region)
        (hw_ready ?from - region ?to - region)

        ; (can_go_to_base)
        ; (has_pulverize_goal)
        ; (has_picture_goal)
        ; (at_move)

    )

   
    (:functions
    

        ;; Variavel q controla bateria em porcentagem
        (battery_amount)
        ;; quantidade de insumo
        (input_amount)
        ;;velocidade de carregar a bateria em porcentagem por segundos
        (recharge_rate_battery)
        ;;velocidade de descarregar a bateria
        (discharge_rate_battery)
        ;;capacidade maxima bateria
        (battery_capacity)
        ;;capacidade maxima de insumo
        (input_capacity)
        ;;velocidade de reabastecer o insumo
        (recharge_rate_input)
        ;;distancia entre regioes em metros
        (distance ?from_region - region ?to_region - region)
        ;;velocidade em m/s
        (velocity)
        (picture_path_len ?region - region)
        (pulverize_path_len ?region - region)
        (total_goals)
        (goals_achived)
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
            ; (> (battery_amount) (+ (* (/ (distance ?from_region ?to_region) (velocity)) (discharge_rate_battery)) 15))
        )
        :effect (and 
                (at start (not (at ?from_region)))
                (at end (been_at ?to_region))
                (at end (at ?to_region))
                (at end (decrease (battery_amount ) 
                      (*
                          (/
                              (distance ?from_region ?to_region)
                              (velocity)
                          )
                          (discharge_rate_battery)
                      )
          
                ))
                (at end(increase (mission_length) (distance ?from_region ?to_region)))
        )
    )
    
    (:durative-action survey
        :parameters (
            ?region - region
        )
        :duration
            (= ?duration 1)
        :condition(and
            (over all(at ?region))
            (over all(picture_goal ?region))
            ; (> (battery_amount) 
            ;     (*
            ;         (/
            ;             1000
            ;             (velocity)
            ;         )
            ;         (discharge_rate_battery)
            ;     )
            ; )
       )
        :effect(and
            (at end(taken_image ?region))
            (at end(increase (mission_length) 1000))
            (at end(decrease (battery_amount) 
                (*
                    (/
                        1000
                        (velocity)
                    )
                    (discharge_rate_battery)
                )
            ))
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
            ; (> (battery_amount) 
            ;     (*
            ;         (/
            ;             314
            ;             (velocity)
            ;         )
            ;         (discharge_rate_battery)
            ;     )
            ; )
       )
        :effect(and
            (at end(pulverized ?region))
            (at end(increase (mission_length) 314))
            (at end(decrease (input_amount) 1))
            (at end(decrease (battery_amount)
                (*
                    (/
                        314
                        (velocity)
                    )
                    (discharge_rate_battery)
                )
            ))
        )
    )
    (:durative-action recharge_battery
        :parameters (?base - base)
        :duration
            (= ?duration 1)
        :condition (and
            (over all(at ?base))
            ;(< (battery_amount) 60)
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
)