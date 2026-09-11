#! /usr/bin/env python3
"""Every key mission.launch.py rewrites must default to the config file's value.

`RewrittenYaml` substitutes its `param_rewrites` keys **unconditionally**. A
launch argument's default is therefore not a fallback - it is an override that
always fires, and it silently beats `param_mission.yaml`. Edit the YAML alone
and nothing changes; the log shows the launch default and the file shows
something else, and the two never meet.

This has now bitten three times:

  * `scan_rotation` stayed at 6.28 rad in the log while the YAML said 5.30.
  * `mission_duration` defaulted to 600.0 against a YAML of 240.0 - on the
    announced 240 s window that is the robot coming home six minutes late,
    which is -70 and the single worst outcome on the scoring table.
  * `stop_after_tags` defaulted to 0 against a YAML of 12, so the early return
    every document described did not exist.

The first was caught by reading a log; the other two by review, on the morning
of the run. So it is pinned here instead: these tests read both files and fail
if any rewritten key disagrees. They need neither ROS nor a graph.
"""

import os
import re
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
LAUNCH = os.path.join(PKG, 'launch', 'mission.launch.py')
PARAMS = os.path.join(PKG, 'config', 'param_mission.yaml')


def rewritten_keys():
    """The keys inside RewrittenYaml(param_rewrites={...}) in mission.launch.py."""
    source = open(LAUNCH).read()
    block = re.search(r'param_rewrites=\{(.*?)\}', source, re.S)
    assert block, 'param_rewrites block not found in mission.launch.py'
    return set(re.findall(r"'([^']+)'\s*:", block.group(1)))


def launch_defaults():
    """{argument name: default_value string} from every DeclareLaunchArgument."""
    source = open(LAUNCH).read()
    out = {}
    for name, default in re.findall(
            r"DeclareLaunchArgument\(\s*'([^']+)'\s*,\s*default_value=\s*'([^']*)'",
            source, re.S):
        out[name] = default
    return out


def mission_params():
    """Every mission-side parameter, whichever node section declares it.

    The rewritten keys are not all mission_control's: optical_correction belongs
    to tag_manager, which runs in the same process under its own node name.
    RewrittenYaml does not care - it substitutes the key wherever it appears -
    so the check must not either.
    """
    with open(PARAMS) as handle:
        cfg = yaml.safe_load(handle)
    merged = {}
    for section in cfg.values():
        if isinstance(section, dict) and 'ros__parameters' in section:
            merged.update(section['ros__parameters'])
    return merged


def test_every_rewritten_key_has_a_launch_argument():
    """A rewrite with no argument substitutes an unset configuration."""
    missing = rewritten_keys() - set(launch_defaults()) - {'use_sim_time',
                                                           'output_directory'}
    assert not missing, 'rewritten but never declared: {}'.format(sorted(missing))


def test_mission_duration_default_matches_the_config():
    """240 s is the announced window.  A 600 here is six minutes late, -70."""
    assert float(launch_defaults()['mission_duration']) == \
        float(mission_params()['mission_duration'])


def test_stop_after_tags_default_matches_the_config():
    """12 is the announced tag count, and the whole basis for stopping early."""
    assert int(launch_defaults()['stop_after_tags']) == \
        int(mission_params()['stop_after_tags'])


def test_scan_rotation_default_matches_the_config():
    assert float(launch_defaults()['scan_rotation']) == \
        float(mission_params()['scan_rotation'])


def test_optical_correction_default_matches_the_config():
    assert launch_defaults()['optical_correction'] == \
        mission_params()['optical_correction']


def test_no_rewritten_numeric_key_silently_disagrees():
    """The general rule, so a newly rewritten key is covered without a new test."""
    defaults = launch_defaults()
    params = mission_params()
    disagreements = []
    for key in sorted(rewritten_keys()):
        if key not in defaults or key not in params:
            continue
        want, got = params[key], defaults[key]
        # YAML gives a real bool, a launch default is always a string, and
        # "False" != "false" is a difference in spelling rather than in meaning.
        if isinstance(want, bool) or str(got).lower() in ('true', 'false'):
            same = str(want).lower() == str(got).lower()
        else:
            try:
                same = abs(float(want) - float(got)) < 1e-9
            except (TypeError, ValueError):
                same = str(want) == str(got)
        if not same:
            disagreements.append('{}: yaml={!r} launch={!r}'.format(key, want, got))
    assert not disagreements, (
        'launch defaults override the config, so these silently win:\n  '
        + '\n  '.join(disagreements))


def test_the_window_is_the_announced_240_seconds():
    """Guards the value itself, not just that the two files agree.

    Both could be edited to the same wrong number; this says what the
    organisers announced. Change it only when they do.
    """
    assert float(mission_params()['mission_duration']) == 240.0


def test_the_tag_count_is_the_announced_twelve():
    assert int(mission_params()['expected_tags']) == 12
    assert int(mission_params()['stop_after_tags']) == 12
