from statemachine import StateMachine, State

class DuploCollectionMachine(StateMachine):
    """
    Duplo collection behavior as an explicit state machine 
    """
    search = State(initial=True)
    align = State()
    approach = State()
    collect = State()

    # foward progression
    search_to_align    = search.to(align)
    align_to_approach  = align.to(approach)
    approach_to_collect = approach.to(collect)
    collect_to_search  = collect.to(search)

    # targetr lost mid-sequence -> back to search (valid only from align/approach)
    lost = align.to(search) | approach.to(search)
