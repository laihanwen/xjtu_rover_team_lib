$taskRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$env:PYTHONPATH = Join-Path $taskRoot 'build/joystick-deps'
$env:SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS = '1'
python (Join-Path $PSScriptRoot 'trial_control_web.py') @args
