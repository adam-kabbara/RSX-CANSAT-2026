## Payload Visualization Shape Design

Goal: update the 2.5D payload orientation canvas so the payload shape matches the current CANSAT artist render instead of the old quadcopter-style visualization.

### Behavior

- The payload body is drawn as a fixed-wing CANSAT silhouette with a curved white fuselage.
- The visualization includes long swept gray wings extending from both sides of the body.
- The nose includes small face-like marker dots and a subtle curved mark, matching the render's visual identity.
- A top support boom and small camera/sensor plate are drawn above the rear/top body area.
- The existing dashed payload midline remains visible and follows the new nose-to-tail body direction.
- Existing telemetry behavior remains unchanged: gyro-estimated orientation, drag-to-view rotation, acceleration vector, speed vector, header, footer, and status text.

### Boundaries

- `code/gui/payload_visualization.py` remains the only implementation file for this change.
- The model stays hand-drawn with PyQt `QPainter` polygons and paths rather than using an image asset.
- Existing projection, rotation, telemetry update, vector drawing, and window wiring stay intact.
- The old quadcopter body, arms, rotor discs, propeller blades, and rotor shadow are removed from the payload drawing path.

### Shape Components

- Fuselage: a rounded, tapered white pod with a heavier black outline and a subtle inner curve.
- Wings: two swept gray panels with dark outlines, attached near the middle of the fuselage.
- Nose marks: two small dark dots and a small curved mark near the front.
- Top boom: a slim support line rising from the body to a small plate.
- Camera plate: a small tilted rectangular plate with a simple camera block and lens circle.
- Shadow: a simplified soft body-and-wing shadow that supports the new aircraft silhouette without implying rotors.

### Validation

- A lightweight unit test should verify that the payload body draw path no longer calls the quadcopter drawing helpers and instead calls the fixed-wing shape helpers.
- Existing data parsing tests should continue to pass.
- A syntax/import check should confirm `code/gui/payload_visualization.py` remains importable.
