from statemachine import StateMachine, State

class DuploCollectionMachine(StateMachine):
    """
    Duplo collection behavior as an explicit state machine 
    """
    search = State(initial=True)
    approach = State()
    collect = State()

    # foward progression
    search_to_approach    = search.to(approach)
    approach_to_collect = approach.to(collect)
    collect_to_search  = collect.to(search)

    # target lost mid-sequence -> back to search (valid only from approach)
    lost = approach.to(search)
