"""Read-only joystick capture: raw values and candidate wire frames, no IO to ROV."""
import argparse
import datetime
import json
import os
from pathlib import Path
import time
from Re_control import encode_frame, joystick_attached


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--wait', type=float, default=0)
    parser.add_argument('--output', default='controller-check.json')
    parser.add_argument('--dial-min', type=float, default=-.5)
    parser.add_argument('--dial-max', type=float, default=-.1)
    args = parser.parse_args()
    if not 0 < args.seconds <= 300 or not 0 <= args.wait <= 300:
        parser.error('seconds must be 0..300; wait must be 0..300')
    os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = '1'
    import pygame
    pygame.display.init()
    pygame.joystick.init()
    report = {'time': datetime.datetime.now().astimezone().isoformat(),
              'read_only': True, 'mapping_verified': False, 'devices': [], 'samples': []}
    try:
        deadline = time.monotonic() + args.wait
        while True:
            pygame.event.pump()
            if pygame.joystick.get_count() or time.monotonic() >= deadline:
                break
            time.sleep(.1)
        devices = []
        for index in range(pygame.joystick.get_count()):
            device = pygame.joystick.Joystick(index)
            if not device.get_init():
                device.init()
            devices.append(device)
            report['devices'].append({'name': device.get_name(), 'guid': device.get_guid(),
                'axes': device.get_numaxes(), 'buttons': device.get_numbuttons(),
                'hats': device.get_numhats()})
        print(json.dumps(report['devices'], ensure_ascii=False), flush=True)
        if not devices:
            report['error'] = 'No joystick recognized by SDL'
        else:
            print('Move one control at a time. Raw values and candidate frames are recorded; nothing is sent.', flush=True)
            start = time.monotonic()
            previous = None
            while time.monotonic() - start < args.seconds:
                pygame.event.pump()
                device = devices[0]
                if not joystick_attached(pygame, device):
                    report['error'] = 'Joystick disconnected'
                    break
                axes = [round(device.get_axis(i), 5) for i in range(device.get_numaxes())]
                buttons = [device.get_button(i) for i in range(device.get_numbuttons())]
                hats = [device.get_hat(i) for i in range(device.get_numhats())]
                sample = {'sec': round(time.monotonic()-start, 3), 'axes': axes,
                          'buttons': buttons, 'hats': hats}
                try:
                    sample['candidate_frame'] = encode_frame(axes, buttons, args.dial_min, args.dial_max).hex(' ')
                except ValueError as error:
                    sample['mapping_error'] = str(error)
                report['samples'].append(sample)
                values = (axes, buttons, hats)
                if values != previous:
                    print(json.dumps(sample, ensure_ascii=False), flush=True)
                    previous = values
                time.sleep(.02)
            if report['samples']:
                count = len(report['samples'][0]['axes'])
                report['axis_ranges'] = [{'axis': i,
                    'min': min(s['axes'][i] for s in report['samples']),
                    'max': max(s['axes'][i] for s in report['samples'])} for i in range(count)]
    finally:
        pygame.quit()
        Path(args.output).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    return 1 if 'error' in report else 0


if __name__ == '__main__':
    raise SystemExit(main())
