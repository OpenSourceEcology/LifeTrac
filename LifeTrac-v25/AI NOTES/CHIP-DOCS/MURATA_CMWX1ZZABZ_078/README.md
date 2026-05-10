# Murata CMWX1ZZABZ-078

## Scope
- Module-level pin mapping and integration constraints
- Internal STM32L072 and SX1276 routing assumptions
- Carrier-board-specific connectivity constraints

## Add Here
- `docs/` module datasheet, hardware design guide, pin map
- `notes/` board-level mapping analysis and contradictions
- `findings.md` confirmed module integration facts
- `open_questions.md` unresolved module-path assumptions

## Immediate Checklist
- Confirm which module pins are actually routed on Max Carrier
- Confirm whether ROM mode and user mode can use different observable UART behavior on same net
