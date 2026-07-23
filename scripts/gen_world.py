#!/usr/bin/env python3
"""Пересобирает building.sdf из config/buildings.yaml.

Все модели с именами из конфига вырезаются и вставляются заново, поэтому
сцену можно перегенерировать сколько угодно раз.
"""
import os
import re
import sys

import yaml

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'avoidance_sim')
CFG = os.path.normpath(os.path.join(PKG, 'config', 'buildings.yaml'))
WORLD = os.path.normpath(os.path.join(PKG, 'worlds', 'building.sdf'))

TPL = '''
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {hz} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material>
            <ambient>{c} 1</ambient><diffuse>{c} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
'''
COLORS = ['0.75 0.35 0.25', '0.45 0.50 0.60', '0.55 0.55 0.40']


def main():
    items = yaml.safe_load(open(CFG))['buildings']
    s = open(WORLD).read()

    # вырезаем старые здания: и по именам из конфига, и прежнее "building"
    names = [b['name'] for b in items] + ['building']
    for n in names:
        s = re.sub(r'\n?[ \t]*<model name="%s">.*?</model>\n?' % re.escape(n),
                   '\n', s, flags=re.S)

    blocks = ''
    for i, b in enumerate(items):
        blocks += TPL.format(c=COLORS[i % len(COLORS)], hz=b['sz'] / 2.0, **b)

    s = s.replace('</world>', blocks + '</world>')
    open(WORLD, 'w').write(s)

    print('зданий в сцене: %d' % len(items))
    for b in items:
        print('  %-4s центр (%.1f, %.1f)  %.0fx%.0f  высота %.0f м'
              % (b['name'], b['x'], b['y'], b['sx'], b['sy'], b['sz']))
    if s.count('<model name=') == 0:
        print('ВНИМАНИЕ: в мире не осталось моделей'); sys.exit(1)


if __name__ == '__main__':
    main()
