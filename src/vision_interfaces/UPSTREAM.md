The two phase-1 service schemas were mirrored verbatim from
https://github.com/Azeem1010/three-camera-vision-ros2/tree/main/vision_interfaces/srv
on 2026-10-22. No perception implementation is included or changed.

- GetPickTarget.srv blob: 072d92a9e775a2a1643103e6d57a21d634ade327
- GetZoneState.srv blob: 2a986a14673d46f2063844ed0b420b66533aab67

When using the vision team's complete vision_interfaces package, exclude this
local package using COLCON_IGNORE and build against their version. Never build
two packages with the same name in one workspace. Update schemas together with
the vision team when their contract changes. No upstream license was provided
in the inspected file tree; these schemas are retained for the team's integration.

B→C update (2026-10-22): also mirrored the following existing schemas verbatim
from the supplied local `three-camera-vision-ros2-main/vision_interfaces/srv` snapshot.
These are existing vision-team APIs, not new invented service contracts.

- GetGridState.srv SHA-256: `4499a26021329006a6b27f06f250729edd6a7b5feae1318d915201860c5e1daf`

- VerifyPlace.srv SHA-256: `c5608e1d65245158a225e29f1ec70c56a9f006ab0b7a741017d6b0cd10616fe9`
