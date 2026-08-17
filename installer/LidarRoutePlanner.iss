; Inno Setup script — LiDAR Route Planner (per-user install, no admin required).
;
; Wraps the PyInstaller ONEDIR bundle (dist\LidarRoutePlanner\, produced by build.ps1)
; into a single Setup.exe that installs to %LOCALAPPDATA%\Programs, adds Start-Menu and
; (optional) Desktop shortcuts, and registers an uninstaller. The target machine needs
; NO Python — the bundle is self-contained.
;
; Build (needs Inno Setup 6 on the build machine):
;     ..\make_installer.ps1                 ; runs ISCC for you, or:
;     iscc installer\LidarRoutePlanner.iss  ; from the repo root
; Override the version / source with:  iscc /DAppVersion=1.2.0 /DSourceDir=<path> ...

#define AppName    "LiDAR Route Planner"
#define AppExe     "LidarRoutePlanner.exe"
#define AppPublisher "LiDAR Route Planner"
#ifndef AppVersion
  #define AppVersion "1.0.0"
#endif
; The frozen onedir bundle to package. Absolute path preferred (make_installer.ps1
; passes one); the default is relative to this .iss file.
#ifndef SourceDir
  #define SourceDir "..\dist\LidarRoutePlanner"
#endif

[Setup]
; Stable AppId so upgrades replace, and uninstall is tracked (keep this constant).
AppId={{7C9E6A2D-4B3F-4E8A-9C1D-2F5B8A0E6D14}
AppName={#AppName}
AppVersion={#AppVersion}
AppPublisher={#AppPublisher}
; Per-user, no elevation. Installs under the user's LocalAppData\Programs.
PrivilegesRequired=lowest
DefaultDirName={localappdata}\Programs\LidarRoutePlanner
DefaultGroupName={#AppName}
DisableProgramGroupPage=yes
DisableDirPage=auto
ArchitecturesInstallIn64BitMode=x64compatible
; Output Setup.exe location + name.
OutputDir=..\dist\installer
OutputBaseFilename=LidarRoutePlanner-Setup-{#AppVersion}
SetupIconFile=..\assets\app.ico
UninstallDisplayIcon={app}\{#AppExe}
UninstallDisplayName={#AppName} {#AppVersion}
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
; ── Updating an existing install (same AppId) ──
; If the app is running, close it (its files are locked) and don't relaunch it.
CloseApplications=yes
RestartApplications=no

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

; On an UPGRADE, wipe the old program folder first so files that a newer PyInstaller
; build no longer ships (its DLL/pyd set changes between versions) can't linger. Safe:
; the app keeps NO user data here — settings live in the registry (QSettings), and the
; DTM/route/LAS files the operator opens live wherever they chose.
[InstallDelete]
Type: filesandordirs; Name: "{app}\*"

[Tasks]
Name: "desktopicon"; Description: "Create a &desktop shortcut"; GroupDescription: "Additional icons:"

[Files]
; The whole self-contained bundle (app exe, its DLLs, and helios\ if build.ps1 bundled it).
Source: "{#SourceDir}\*"; DestDir: "{app}"; Flags: recursesubdirs createallsubdirs ignoreversion

[Icons]
Name: "{group}\{#AppName}"; Filename: "{app}\{#AppExe}"
Name: "{group}\Uninstall {#AppName}"; Filename: "{uninstallexe}"
Name: "{autodesktop}\{#AppName}"; Filename: "{app}\{#AppExe}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#AppExe}"; Description: "Launch {#AppName}"; Flags: nowait postinstall skipifsilent
