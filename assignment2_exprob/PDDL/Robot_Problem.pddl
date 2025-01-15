(define (problem Robot_Problem) (:domain Robot_Domain)
(:objects
    loc0 loc1 loc2 loc3 loc4 - location
    tgt1 tgt2 tgt3 tgt4 - target
    RosBot - robot
)

(:init
    (not_visited loc0)
    (not_visited loc1)
    (not_visited loc2)
    (not_visited loc3)
    (not_visited loc4)

    (target_at tgt1 loc0)
    (target_at tgt2 loc1)
    (target_at tgt3 loc2)
    (target_at tgt4 loc3)

    (not_detected tgt1)
    (not_detected tgt2)
    (not_detected tgt3)
    (not_detected tgt4)

    (robot_at RosBot loc4)

   

)

(:goal (and
    (visited loc0)
    (visited loc1)
    (visited loc2)
    (visited loc3)
    

    

    (detected tgt1)
    (detected tgt2)
    (detected tgt3)
    (detected tgt4)

    (all_targets_detected)

    
))

;un-comment the following line if metric is needed
;(:metric minimize (???))
)
