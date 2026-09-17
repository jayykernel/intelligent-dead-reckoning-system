"""
IO-VNBD Fixed Dataset Split Specification.

Divides the 72 sessions across drivers into reproducible Train, Validation, and Test sets.
"""

# Hold out entire drivers and distinct routes for testing to evaluate true generalization
# Driver A: S1..S4 (diverse daily driving) -> S1, S2, S3a for Train; S3b, S3c for Val; S4 for Test
# Driver B: M -> Train
# Driver D: Y1 -> Val
# Driver E (Vf): Vfa01 (Train), Vfa02 (Test)
# Driver E (Vta): Vta01a..Vta20 (Train), Vta21..Vta25 (Val), Vta26..Vta30 (Test)
# Driver E (Vtb): Vtb01..Vtb08 (Train), Vtb09..Vtb10 (Val), Vtb11..Vtb12 (Test)
# Driver E (Vw): Vw01..Vw12 (Train), Vw13..Vw14c (Val), Vw15..Vw17 (Test)

TRAIN_SESSIONS = [
    ("S (Driver A)", "S1"),
    ("S (Driver A)", "S2"),
    ("S (Driver A)", "S3a"),
    ("M (Driver B)", "M"),
    ("Vf (Driver E)", "V-Vfa01"),
    ("Vta (Driver E)", "Vta01a"),
    ("Vta (Driver E)", "Vta01b"),
    ("Vta (Driver E)", "Vta02"),
    ("Vta (Driver E)", "Vta03"),
    ("Vta (Driver E)", "Vta04"),
    ("Vta (Driver E)", "Vta05"),
    ("Vta (Driver E)", "Vta06"),
    ("Vta (Driver E)", "Vta07"),
    ("Vta (Driver E)", "Vta08"),
    ("Vta (Driver E)", "Vta09"),
    ("Vta (Driver E)", "Vta10"),
    ("Vta (Driver E)", "Vta11"),
    ("Vta (Driver E)", "Vta12"),
    ("Vta (Driver E)", "Vta13"),
    ("Vta (Driver E)", "Vta14"),
    ("Vta (Driver E)", "Vta15"),
    ("Vta (Driver E)", "Vta16"),
    ("Vta (Driver E)", "Vta17"),
    ("Vta (Driver E)", "Vta19"),
    ("Vta (Driver E)", "Vta20"),
    ("Vtb (Driver E)", "Vtb01"),
    ("Vtb (Driver E)", "Vtb02"),
    ("Vtb (Driver E)", "Vtb03"),
    ("Vtb (Driver E)", "Vtb04"),
    ("Vtb (Driver E)", "Vtb05"),
    ("Vtb (Driver E)", "Vtb06"),
    ("Vtb (Driver E)", "Vtb07"),
    ("Vtb (Driver E)", "Vtb08"),
    ("Vw (Driver E)", "Vw01"),
    ("Vw (Driver E)", "Vw02"),
    ("Vw (Driver E)", "Vw03"),
    ("Vw (Driver E)", "Vw04"),
    ("Vw (Driver E)", "Vw05"),
    ("Vw (Driver E)", "Vw06"),
    ("Vw (Driver E)", "Vw07"),
    ("Vw (Driver E)", "Vw08"),
    ("Vw (Driver E)", "Vw09"),
    ("Vw (Driver E)", "Vw10"),
    ("Vw (Driver E)", "Vw11"),
    ("Vw (Driver E)", "Vw12"),
]

VAL_SESSIONS = [
    ("S (Driver A)", "S3b"),
    ("S (Driver A)", "S3c"),
    ("Y (Driver D)", "Y1"),
    ("Vta (Driver E)", "Vta21"),
    ("Vta (Driver E)", "Vta22"),
    ("Vta (Driver E)", "Vta23"),
    ("Vta (Driver E)", "Vta24"),
    ("Vta (Driver E)", "Vta25"),
    ("Vtb (Driver E)", "Vtb09"),
    ("Vtb (Driver E)", "Vtb10"),
    ("Vw (Driver E)", "Vw13"),
    ("Vw (Driver E)", "Vw14a"),
    ("Vw (Driver E)", "Vw14b"),
    ("Vw (Driver E)", "Vw14c"),
]

TEST_SESSIONS = [
    ("S (Driver A)", "S4"),
    ("Vf (Driver E)", "V-Vfa02"),
    ("Vta (Driver E)", "Vta26"),
    ("Vta (Driver E)", "Vta27"),
    ("Vta (Driver E)", "Vta28"),
    ("Vta (Driver E)", "Vta29"),
    ("Vta (Driver E)", "Vta30"),
    ("Vtb (Driver E)", "Vtb11"),
    ("Vtb (Driver E)", "Vtb12"),
    ("Vw (Driver E)", "Vw15"),
    ("Vw (Driver E)", "Vw16a"),
    ("Vw (Driver E)", "Vw16b"),
    ("Vw (Driver E)", "Vw17"),
]

DATASET_SPLITS = {
    "train": TRAIN_SESSIONS,
    "val": VAL_SESSIONS,
    "test": TEST_SESSIONS
}
