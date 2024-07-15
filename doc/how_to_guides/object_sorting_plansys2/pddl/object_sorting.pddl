(define (domain object_sorting)
(:requirements :strips :typing :adl :fluents :durative-actions)
;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
manipulator
pose
object
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;


(:predicates 
    
(manipulator_at ?m - manipulator ?p - pose)
(connected ?p1 ?p2 - pose)
(object_at ?o - object ?p - pose)
(object_at_manipulator ?o - object ?m - manipulator)

)


(:functions 
)

(:durative-action move_it
    :parameters (?m - manipulator ?p1 ?p2 - pose)
    :duration ( = ?duration 1)
    :condition (and 
        (at start (manipulator_at ?m ?p1))
        (at start (connected ?p1 ?p2))
        )
    :effect (and 
        (at start (not(manipulator_at ?m ?p1)))
        (at end (manipulator_at ?m ?p2))
    )
)

(:durative-action pick
    :parameters (?m - manipulator ?o - object ?p - pose)
    :duration (= ?duration 1)
    :condition (and 
        (at start (manipulator_at ?m ?p))
        (at start (object_at ?o ?p))
    )
    :effect (and 
        (at start (not(object_at ?o ?p)))
        (at end (object_at_manipulator ?o ?m))
    )
)

(:durative-action place
    :parameters (?m - manipulator ?o - object ?p - pose)
    :duration (= ?duration 1)
    :condition (and 
        (at start (manipulator_at ?m ?p))
        (at start (object_at_manipulator ?o ?m))
    )
    :effect (and 
        (at start (not(object_at_manipulator ?o ?m)))
        (at end (object_at ?o ?p))
    )
)

)