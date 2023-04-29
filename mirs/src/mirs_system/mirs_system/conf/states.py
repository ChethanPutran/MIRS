class States:

    # General
    INACTIVE = 'state_inactive'
    ACTIVE = 'state_active'
    DONE = "state_done"
    ERROR = "state_error"
    WARNING = "state_warning"
    PAUSE = "state_pause"
    STOP = "state_stop"

    # Recorder
    RECORDING = 'state_recording'
    RECORDED = 'state_recorded'
    RECIEVE_RECORING = 'state_recieve_recording'

    # Executor
    EXECUTING = "state_executing"
    EXECUTED = "state_executed"

    # Extractor
    EXTRACTED = 'state_extracted'
    EXTRACTING = 'state_extracting'
    RECIEVE_TASKS = 'state_recieve_tasks'
