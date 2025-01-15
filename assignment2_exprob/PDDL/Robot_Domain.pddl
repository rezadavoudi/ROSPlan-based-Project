(define (domain Robot_Domain)

;remove requirements that are not needed
(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types 
    robot
    location
    target
)

; un-comment following line if constants are needed
;(:constants )

(:predicates 
    (robot_at ?r - robot ?l - location)
    (target_at ?t - target ?l - location)
    (visited ?l - location)
    (not_visited ?l - location)
    (detected ?t - target)
    (not_detected ?t - target)
    (all_targets_detected)
)

;(:functions ;todo: define numeric functions here
;)

;define actions here

(:action move_to
    :parameters (?r - robot ?from ?to - location)
    :precondition (and 
                    (robot_at ?r ?from)
                    (not_visited ?to)
                    
    )
    :effect (and 
                
                (not (not_visited ?to))
                (visited ?to)
                (robot_at ?r ?to)
                (not (robot_at ?r ?from))
    )
)

(:action identify
    :parameters (?r - robot ?l - location ?t - target)
    :precondition (and 
                    (robot_at ?r ?l)
                    (target_at ?t ?l)
                    (not_detected ?t)
    )
    :effect (and 
                
                (not (not_detected ?t))
                (detected ?t)
    )
)

(:action verify_targets ; Checks to see if all targets are detected
    :parameters ()
    :precondition (and 
                    (forall (?t - target) (detected ?t)) ; All targets must be detected
    )
    :effect (and 
                (all_targets_detected) ; Set the predicate indicating all targets are detected
    )
)

)

