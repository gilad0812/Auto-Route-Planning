"""App version, shown in the window title. CI overwrites this file before a build
(a release tag v1.2.0 -> '1.2.0'; any other build -> 'dev+<commit>'), so the .exe on
the standalone machine always says which build it is."""
__version__ = 'dev'
