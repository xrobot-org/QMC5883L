# QMC5883L

## Static assembly source line

This source line uses explicit C++ constructor dependencies and ordered instance
arguments. Inspect the current primary header with `xrobot_mod_parser --path .`;
its declarations, not old manifest/config examples, define the interface.
Historical HardwareContainer/ApplicationManager examples below apply only to the
older dynamic source tags. Device/protocol descriptions remain relevant.
See the XRobot [migration guide](https://github.com/xrobot-org/XRobot/blob/dev/MIGRATION.md).
Compilation is not hardware validation; retain version-specific board evidence.


LibXR 框架下的 QMC5883L 三轴磁力计驱动模块。

## Required Hardware
- i2c_qmc5883l
- qmc5883l_int
- ramfs

## Constructor Arguments
- rotation: LibXR::Quaternion<float>
- topic_name: const char*
- task_stack_depth: size_t

## Template Arguments
None

## Depends
None
