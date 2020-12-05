Header header

# phase of ID change
uint8 phase
# security ID a.k.a. pseudonym
uint8[8] id

uint8 PHASE_PREPARE = 1 # nodes shall prepare for upcoming ID change
uint8 PHASE_COMMIT = 2  # new ID is now valid
uint8 PHASE_ABORT = 3   # scheduled ID change has been aborted