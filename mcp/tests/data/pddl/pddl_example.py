domain = """
(define (domain blocksworld)
  (:requirements :strips :typing)
  (:types block)

  (:predicates
    (on ?x ?y - block)      ; Block ?x is on top of block ?y
    (ontable ?x - block)    ; Block ?x is on the table
    (clear ?x - block)      ; Nothing is on top of block ?x
    (handempty)             ; The robot arm is empty
    (holding ?x - block)    ; The robot arm is holding block ?x
  )

  (:action pick-up
    :parameters (?x - block)
    :precondition (and (clear ?x) (ontable ?x) (handempty))
    :effect (and (not (ontable ?x)) (not (clear ?x)) (not (handempty)) (holding ?x))
  )

  (:action put-down
    :parameters (?x - block)
    :precondition (holding ?x)
    :effect (and (not (holding ?x)) (clear ?x) (handempty) (ontable ?x))
  )

  (:action stack
    :parameters (?x ?y - block)
    :precondition (and (holding ?x) (clear ?y))
    :effect (and (not (holding ?x)) (not (clear ?y)) (clear ?x) (handempty) (on ?x ?y))
  )

  (:action unstack
    :parameters (?x ?y - block)
    :precondition (and (on ?x ?y) (clear ?x) (handempty))
    :effect (and (not (on ?x ?y)) (not (clear ?x)) (clear ?y) (holding ?x) (not (handempty)))
  )
)
"""

problem = """
(define (problem p01)
  (:domain blocksworld)
  (:objects A B - block) ; Two blocks, A and B
  (:init
    (ontable A)     ; Block A is on the table
    (on B A)        ; Block B is on top of block A
    (clear B)       ; Nothing is on top of B
    (handempty)     ; The arm is empty
  )
  (:goal (and
    (on A B)        ; Block A should be on block B
    (clear A)       ; Block A should be clear
    (ontable B)     ; Block B should be on the table
  ))
)
"""
