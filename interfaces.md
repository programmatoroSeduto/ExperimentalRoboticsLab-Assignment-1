

# cluedo_movement_controller

## Services

- `/go_to` : `robocluedo_msgs/GoTo`
    
    ```yaml
    ## file 'GoTo.xrv'
    
    # request: where to go
    string where
    
    ---
    
    # response: true if the place is reachable
    # STUB IMPLEMENTATION: always true
    bool success
    ```
    

## Publishers

- `/hint_signal` : `td_msgs/Empty`

---

# cluedo_random_room

## Services

- `/random_room` : `robocluedo_msgs/RandomRoom`
    
    ```yaml
    ## file 'RandomRoom.srv'
    
    # empty request
    ---
    
    # reply: the next room
    string room
    ```
    

## Parameter server

- (GET) `cluedo_path_where` : path of the textfile with all the names of the places

---

# cluedo_armor_interface

test necessario: usa Python per fare i test in questo caso. 

## Services

- `/cluedo_armor/add_hint` : `robocluedo_msgs/AddHint`
    
    ```yaml
    ## file 'AddHint.srv'
    # it represents a hint of the type -> (Aelem, Belem):property
    
    # the numeric ID of the hint
    int32 hypID
    
    # fields of the property
    string property
    string Aelem
    string Belem
    
    ---
    
    bool success
    ```
    
- `/cluedo_armor/find_consistent_h` : `robocluedo_msgs/FindConsistentHypotheses.h`
    
    ```yaml
    ## file 'FindConsistentHypotheses.srv'
    
    # empty request
    
    ---
    
    # list of consistent hypotheses from the ontology
    Hypothesis[] hyp
    ```
    
    message type `Hypothesis` : 
    
    ```yaml
    ## file 'Hypothesis.msg'
    
    # the tag of the hypothesis
    string tag
    # who killed Dr. Black
    string who
    # where the murder happened
    string where
    # what is the murder weapon
    string what
    ```
    
- `/cluedo_armor/wrong_hypothesis` : `robocluedo_msgs/DiscardHypothesis.h`
    
    ```yaml
    ## file 'DiscardHypothesis.srv'
    
    string hypothesisTag
    
    ---
    
    bool success
    ```

- `/cluedo_armor/backup` : `std_srvs/Trigger`
    

## Parameter server

- (GET) `cluedo_path_owlfile`

---

# cluedo_oracle - TEST

testare questo nodo! è stato reimplementato, dunque va testato. 

## Services

- `/check_solution` : `robocluedo_msgs/CheckSolution.h`
    
    ```yaml
    ## file 'CheckSolution.srv'
    
    # who killed Dr Black
    string Who
    # Where Dr Black was killed
    string Where
    # What is the murder weapon
    string What
    
    ---
    
    # is the solution correct?
    bool MysterySolved
    ```
    

## Publishers

- `/hint` : `robocluedo_msgs/Hint`
    
    ```yaml
    ## file 'Hint.msg'
    
    # the ID of the hint
    int32 HintID
    # the type of hint from the oracle
    string HintType
    # the content of the hint
    string HintContent
    ```
    

## Subscribers

- `/hint_signal` : `std_msgs/Empty`

## Parameter server

- (GET) `cluedo_path_where` : file path
- (GET) `cluedo_path_what` : file path
- (GET) `cluedo_path_who` : file path

---

# robocluedo_main - TEST

## Clients

- `/go_to` : `robocluedo_msgs/GoTo`
- `/random_room` : `robocluedo_msgs/RandomRoom`
- `/check_solution` : `robocluedo_msgs/CheckSolution`
- `/cluedo_armor/add_hint` : `robocluedo_msgs/AddHint`
- `/cluedo_armor/find_consistent_h` : `robocluedo_msgs/FindConsistentHypotheses`
- `/cluedo_armor/wrong_hypothesis` : `robocluedo_msgs/DiscardHypothesis`

## Subscribers

- `/hint` : `robocluedo_msgs/Hint`

---
