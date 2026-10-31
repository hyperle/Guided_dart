"""Storage script regression checks; all writes stay in temporary fake boards."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

REPO = Path(__file__).resolve().parents[1]
USB = 'gphoto2://Kendryte_CanMV_001000000'


class RtosScriptsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='dart-storage-test-')
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.repo = self.root / 'repo'
        shutil.copytree(REPO / '.script', self.repo / '.script',
                        ignore=shutil.ignore_patterns('board.env', '__pycache__'))
        source_list = self.repo / 'runtime/launcher/startup.list'
        source_list.parent.mkdir(parents=True)
        source_list.write_text('/sdcard/app/dart_app &\n')
        self.sd = self.root / 'sd'
        self.data = self.root / 'data'
        self.keep = [
            'sd/revision.txt', 'sd/app/launcher', 'sd/app/dart_app',
            'sd/app/startup_final.list', 'data/app/launcher',
            'data/app/logs/launcher.pid', 'data/app/logs/dart_app.done',
            'sd/app/isp/calibration.log', 'data/app/config.csv',
            'outside/untouched.log',
        ]
        self.logs = [
            'sd/app/logs/old.log', 'data/app/logs/imu_000001.log',
            'data/app/logs/.hidden', 'data/app/logs/name with spaces.log',
            'data/app/logs-before-migration/saved.txt',
            'data/app/archive-sdcard/app.log', 'data/app/archive/imu.log.1',
        ]
        self.records = [
            'sd/app/records/rec_0001/img/a.png',
            'data/app/records/rec_0002/img/b0001/p0001/frame.png',
            'data/app/records/rec_0002/logs/frames.csv',
            'data/app/records/detection.csv',
            'data/app/archive-sdcard/records/rec_0003/raw.bin',
            'data/app/archive-sdcard/detect.csv', 'data/app/archive/movie.mp4',
        ]
        for name in self.keep + self.logs + self.records:
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(name)
        (self.data / 'app/logs/linked.log').symlink_to(self.root / 'outside/untouched.log')
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        gio = self.bin / 'gio'
        gio.write_text('''#!/usr/bin/python3
import os, subprocess, sys
from pathlib import Path
from urllib.parse import unquote, urlsplit
args = sys.argv[1:]
for i, arg in enumerate(args):
    if arg.startswith(('gphoto2://', 'mtp://')):
        parts = unquote(urlsplit(arg).path).strip('/').split('/')
        store = {'store_ffff0001': 'sd', 'store_ffff0002': 'data'}.get(parts[0])
        if not store:
            sys.exit(1)
        args[i] = str(Path(os.environ['FAKE_BOARD']) / store / '/'.join(parts[1:]))
if args[0] == 'list' and os.environ.get('FAIL_LIST') and args[-1].endswith(os.environ['FAIL_LIST']):
    print('simulated storage read error', file=sys.stderr)
    sys.exit(42)
if args[0] == 'info' and os.environ.get('FAIL_INFO') and args[-1].endswith(os.environ['FAIL_INFO']):
    sys.exit(42)
if args[0] == 'remove' and os.environ.get('BUSY_REMOVE') and args[-1].endswith(os.environ['BUSY_REMOVE']):
    print('simulated file busy', file=sys.stderr)
    sys.exit(1)
if args[0] == 'remove' and os.environ.get('MUTATE_ON_REMOVE'):
    marker = Path(os.environ['FAKE_BOARD']) / 'mutated'
    if not marker.exists():
        with (Path(os.environ['FAKE_BOARD']) / os.environ['MUTATE_ON_REMOVE']).open('a') as f:
            f.write('late write')
        marker.touch()
sys.exit(subprocess.call(['/usr/bin/gio', *args]))
''')
        gio.chmod(0o755)
        ssh = self.bin / 'ssh'
        ssh.write_text('#!/usr/bin/python3\nimport subprocess, sys\nsys.exit(subprocess.call(["bash", "-c", sys.argv[-1]]))\n')
        ssh.chmod(0o755)
        self.env = dict(os.environ, PATH=f'{self.bin}:{os.environ["PATH"]}',
                        BOARD_IP='', SDCARD=str(self.sd), DATA_SDCARD=str(self.data),
                        BOARD_DIR='/sdcard/app', BOARD_DATA_DIR='/data/app',
                        FAKE_BOARD=str(self.root), NO_COLOR='1', RTOS_IDLE_CHECK_SECONDS='1')
        for key in ('FAIL_LIST', 'FAIL_INFO', 'BUSY_REMOVE', 'MUTATE_ON_REMOVE'):
            self.env.pop(key, None)

    def run_script(self, name, *args, ok=True):
        result = subprocess.run(['bash', str(self.repo / '.script' / name), *args],
                                env=self.env, text=True, capture_output=True)
        if ok:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        else:
            self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        return result

    def mode(self, mode):
        if mode == 'mtp':
            self.env.update(SDCARD=f'{USB}/store_ffff0001', DATA_SDCARD='')
        elif mode == 'ssh':
            self.env.update(BOARD_IP='fake-board', BOARD_DIR=str(self.sd / 'app'),
                            BOARD_DATA_DIR=str(self.data / 'app'))

    def assert_present(self, names):
        for name in names:
            self.assertEqual((self.root / name).read_text(), name)

    def run_during_observation(self, mutate, *args):
        with subprocess.Popen(['bash', str(self.repo / '.script/clean-rtos-data'), *args],
                              env=self.env, text=True, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE) as proc:
            lines = []
            while line := proc.stdout.readline():
                lines.append(line)
                if '观察写入变化' in line:
                    mutate()
                    break
            else:
                self.fail('cleanup did not reach the observation window')
            stdout, stderr = proc.communicate(timeout=60)
        output = ''.join(lines) + stdout
        self.assertEqual(proc.returncode, 0, output + stderr)
        return output

    def check_cleanup(self, mode):
        self.mode(mode)
        self.run_script('clean-rtos-data')
        self.assert_present(self.keep)
        self.assertTrue((self.data / 'app/logs/linked.log').is_symlink())
        for name in self.logs + self.records:
            self.assertFalse((self.root / name).exists(), name)
        self.assertFalse((self.data / 'app/records/rec_0002').exists())
        self.run_script('clean-rtos-data')  # Repeated cleanup is harmless.

    def test_cleanup_card(self):
        self.check_cleanup('card')

    def test_cleanup_mtp(self):
        self.check_cleanup('mtp')

    def test_cleanup_ssh(self):
        self.check_cleanup('ssh')

    def test_write_switch_and_cleanup_preserve_manual_pause(self):
        for mode in ('card', 'mtp', 'ssh'):
            with self.subTest(mode=mode):
                self.env.update(BOARD_IP='', SDCARD=str(self.sd), DATA_SDCARD=str(self.data),
                                BOARD_DIR='/sdcard/app', BOARD_DATA_DIR='/data/app')
                self.mode(mode)
                self.run_script('write-rtos', 'off')
                self.run_script('write-rtos', 'off')
                pause = self.sd / 'app/control/write.pause'
                self.assertTrue((pause / 'manual').exists())
                result = self.run_script('clean-rtos-data')
                self.assertIn('不等待板端应答', result.stdout)
                self.assertEqual([p.name for p in pause.iterdir()], ['manual'])
                self.run_script('write-rtos', 'on')
                self.assertFalse(pause.exists())

    def test_cleanup_requests_pause_before_observation_and_keeps_failed_pause(self):
        self.mode('mtp')
        pause = self.sd / 'app/control/write.pause'
        active = self.data / 'app/logs/imu_000001.log'
        def writing():
            self.assertTrue(pause.is_dir())
            self.assertTrue(any(p.name.startswith('cleanup.') for p in pause.iterdir()))
            active.write_text('still closing')
        output = self.run_during_observation(writing)
        self.assertIn('保持暂停', output)
        self.assertTrue(pause.is_dir())

    def test_successful_cleanup_can_keep_pause(self):
        self.run_script('clean-rtos-data', '--keep-paused')
        self.assertTrue((self.sd / 'app/control/write.pause').is_dir())
        self.run_script('write-rtos', 'on')
        self.assertFalse((self.sd / 'app/control/write.pause').exists())

    def test_dry_run(self):
        self.mode('mtp')
        result = self.run_script('clean-rtos-data', '--dry-run')
        self.assertIn('待删除', result.stdout)
        self.assert_present(self.keep + self.logs + self.records)

    def test_growing_mtp_log_is_skipped(self):
        self.mode('mtp')
        path = self.data / 'app/logs/imu_000001.log'
        output = self.run_during_observation(lambda: path.write_text(path.read_text() + 'new sample'))
        self.assertIn('跳过（正在写入', output)
        self.assertTrue(path.read_text().endswith('new sample'))
        self.assertFalse((self.sd / 'app/logs/old.log').exists())

    def test_same_size_mtp_overwrite_is_skipped(self):
        self.mode('mtp')
        path = self.data / 'app/logs/imu_000001.log'
        content = 'x' * path.stat().st_size
        output = self.run_during_observation(lambda: path.write_text(content))
        self.assertIn('跳过（正在写入', output)
        self.assertEqual(path.read_text(), content)

    def test_nested_frame_write_preserves_entire_recording(self):
        self.mode('mtp')
        recording = self.data / 'app/records/rec_0002'
        frame = recording / 'img/b0001/p0001/frame.png'
        unchanged_sibling = recording / 'logs/frames.csv'
        root_mtime = recording.stat().st_mtime_ns
        self.run_during_observation(lambda: frame.write_text('updated frame'))
        self.assertEqual(recording.stat().st_mtime_ns, root_mtime)
        self.assertEqual(frame.read_text(), 'updated frame')
        self.assertTrue(unchanged_sibling.exists())
        self.assertFalse((self.sd / 'app/records/rec_0001').exists())

    def test_new_nested_frame_preserves_entire_recording(self):
        recording = self.data / 'app/records/rec_0002'
        frame = recording / 'img/b0001/p0001/new.png'
        root_mtime = recording.stat().st_mtime_ns
        self.run_during_observation(lambda: frame.write_text('new frame'))
        self.assertEqual(recording.stat().st_mtime_ns, root_mtime)
        self.assertTrue((recording / 'logs/frames.csv').exists())
        self.assertEqual(frame.read_text(), 'new frame')

    def check_open_writer(self, mode):
        self.mode(mode)
        path = self.data / 'app/records/rec_0002/img/b0001/p0001/frame.png'
        with path.open('a'):
            output = self.run_script('clean-rtos-data').stdout
        self.assertIn('打开的写句柄', output)
        self.assertEqual(path.read_text(), 'data/app/records/rec_0002/img/b0001/p0001/frame.png')
        self.assertTrue((self.data / 'app/records/rec_0002/logs/frames.csv').exists())
        self.assertFalse((self.sd / 'app/records/rec_0001').exists())

    def test_open_writer_card(self):
        self.check_open_writer('card')

    def test_open_writer_ssh(self):
        self.check_open_writer('ssh')

    def test_read_only_handle_does_not_block_cleanup(self):
        path = self.data / 'app/logs/imu_000001.log'
        with path.open('r'):
            self.run_script('clean-rtos-data')
        self.assertFalse(path.exists())

    def test_unreadable_metadata_is_skipped(self):
        self.mode('mtp')
        self.env['FAIL_INFO'] = '/logs/imu_000001.log'
        output = self.run_script('clean-rtos-data').stdout
        self.assertIn('跳过（无法确认空闲）', output)
        self.assertTrue((self.data / 'app/logs/imu_000001.log').exists())
        self.assertFalse((self.sd / 'app/logs/old.log').exists())

    def test_unreadable_descendant_preserves_entire_recording(self):
        self.mode('mtp')
        self.env['FAIL_LIST'] = '/rec_0002/img/b0001/p0001'
        self.run_script('clean-rtos-data')
        self.assertTrue((self.data / 'app/records/rec_0002/logs/frames.csv').exists())
        self.assertTrue((self.data / 'app/records/rec_0002/img/b0001/p0001/frame.png').exists())
        self.assertFalse((self.sd / 'app/records/rec_0001').exists())

    def test_busy_remove_is_skipped_and_cleanup_continues(self):
        self.mode('mtp')
        self.env['BUSY_REMOVE'] = '/logs/imu_000001.log'
        output = self.run_script('clean-rtos-data').stdout
        self.assertIn('跳过（删除被拒绝', output)
        self.assertTrue((self.data / 'app/logs/imu_000001.log').exists())
        self.assertFalse((self.sd / 'app/logs/old.log').exists())

    def test_rechecks_after_previous_deletions(self):
        self.mode('mtp')
        self.env['MUTATE_ON_REMOVE'] = 'sd/app/logs/old.log'
        self.run_script('clean-rtos-data')
        self.assertTrue((self.sd / 'app/logs/old.log').read_text().endswith('late write'))

    def test_dry_run_skips_writes_without_deleting(self):
        self.mode('mtp')
        path = self.data / 'app/logs/imu_000001.log'
        output = self.run_during_observation(lambda: path.write_text('new sample'), '--dry-run')
        self.assertIn('跳过（正在写入', output)
        self.assertIn('未删除任何文件', output)
        self.assert_present(self.keep + self.records)

    def test_invalid_observation_interval_rejected(self):
        self.env['RTOS_IDLE_CHECK_SECONDS'] = '0'
        self.run_script('clean-rtos-data', ok=False)
        self.assert_present(self.keep + self.logs + self.records)

    def test_logs_alias(self):
        self.run_script('clean-rtos', 'logs')
        self.assert_present(self.keep + self.records)
        for name in self.logs:
            self.assertFalse((self.root / name).exists(), name)

    def test_records_alias(self):
        self.run_script('clean-rtos', 'records')
        self.assert_present(self.keep + self.logs)
        for name in self.records:
            self.assertFalse((self.root / name).exists(), name)

    def test_data_partition_required(self):
        self.env['DATA_SDCARD'] = ''
        self.run_script('clean-rtos-data', ok=False)
        self.assert_present(self.keep + self.logs + self.records)

    def test_same_partition_rejected(self):
        self.env['DATA_SDCARD'] = str(self.sd)
        self.run_script('clean-rtos-data', ok=False)
        self.assert_present(self.keep + self.logs + self.records)

    def test_mtp_read_failure_does_not_delete(self):
        self.mode('mtp')
        self.env['FAIL_LIST'] = '/data'
        result = self.run_script('clean-rtos-data', ok=False)
        self.assertIn('simulated storage read error', result.stderr)
        self.assert_present(self.keep + self.logs + self.records)

    def test_busy_scan_directory_is_skipped(self):
        self.mode('mtp')
        self.env['FAIL_LIST'] = '/app/logs'
        result = self.run_script('clean-rtos-data')
        self.assertIn('跳过（目录忙碌或无法读取）', result.stdout)
        self.assert_present(self.keep + [name for name in self.logs if '/app/logs/' in name])
        self.assertFalse((self.sd / 'app/records/rec_0001').exists())

    def test_full_card_deploy_uses_data_partition(self):
        artifacts = self.repo / 'artifacts'
        for name in ('launcher', 'dart_app', 'imu_raw_probe'):
            path = artifacts / name / name
            path.parent.mkdir(parents=True)
            path.write_text('new-' + name)
        self.env['DART_BUILD_DIR'] = str(artifacts)
        self.run_script('deploy-rtos', '--keep-logs')
        self.assertEqual((self.sd / 'app/dart_app').read_text(), 'new-dart_app')
        self.assertEqual((self.sd / 'app/imu_raw_probe').read_text(), 'new-imu_raw_probe')
        self.assert_present(self.logs + self.records)
        self.run_script('deploy-rtos')
        self.assertFalse((self.data / 'app/logs/imu_000001.log').exists())
        self.assertTrue((self.data / 'app/records').is_dir())
        self.assertTrue((self.data / 'app/logs/launcher.pid').is_file())

    def test_custom_mtp_directories_and_full_deploy(self):
        artifacts = self.repo / 'artifacts'
        for name in ('launcher', 'dart_app', 'imu_raw_probe'):
            path = artifacts / name / name
            path.parent.mkdir(parents=True)
            path.write_text('new-' + name)
        self.env.update(DART_BUILD_DIR=str(artifacts), BOARD_DIR='/sdcard/program',
                        BOARD_DATA_DIR='/data/capture')
        self.mode('mtp')
        result = self.run_script('deploy-rtos')
        self.assertIn('部署完成', result.stdout)
        self.assertEqual((self.sd / 'program/dart_app').read_text(), 'new-dart_app')
        self.assertEqual((self.sd / 'program/imu_raw_probe').read_text(), 'new-imu_raw_probe')
        self.assertTrue((self.data / 'capture/logs').is_dir())
        self.assertFalse((self.sd / 'program/logs').exists())
        self.assert_present(self.keep + self.logs + self.records)

    def test_single_card_deploy_does_not_need_data_mount(self):
        self.env['DATA_SDCARD'] = ''
        self.run_script('deploy-rtos', 'list')
        self.assert_present(self.logs + self.records)

    def test_pwm_swing_deploy_is_explicit_and_preserves_existing_files(self):
        for mode in ('card', 'mtp'):
            with self.subTest(mode=mode):
                if mode == 'mtp':
                    self.mode(mode)
                artifacts = self.repo / 'artifacts'
                binary = artifacts / 'pwm_swing/pwm_swing'
                binary.parent.mkdir(parents=True, exist_ok=True)
                binary.write_text('pwm-swing-' + mode)
                self.env.update(DART_BUILD_DIR=str(artifacts), DATA_SDCARD='')
                result = self.run_script('deploy-rtos', 'pwm_swing')
                deployed = self.sd / 'app/pwm_swing'
                self.assertEqual(deployed.read_text(), binary.read_text())
                if mode == 'card':
                    self.assertTrue(os.access(deployed, os.X_OK))
                self.assertIn('/sdcard/app/pwm_swing', result.stdout)
                self.assert_present(self.keep + self.logs + self.records)

    def test_imu_raw_probe_deploy_preserves_files_without_data_mount(self):
        for mode in ('card', 'mtp'):
            with self.subTest(mode=mode):
                if mode == 'mtp':
                    self.mode(mode)
                binary = self.repo / 'artifacts/imu_raw_probe/imu_raw_probe'
                binary.parent.mkdir(parents=True, exist_ok=True)
                binary.write_text('imu-raw-' + mode)
                self.env.update(DART_BUILD_DIR=str(self.repo / 'artifacts'), DATA_SDCARD='')
                result = self.run_script('deploy-rtos', 'imu_raw_probe')
                self.assertEqual((self.sd / 'app/imu_raw_probe').read_text(), binary.read_text())
                self.assertIn('deploy-rtos list', result.stdout)
                self.assert_present(self.keep + self.logs + self.records)

    def test_unsafe_root_rejected(self):
        self.env['SDCARD'] = '/'
        self.run_script('clean-rtos-data', ok=False)
        self.assert_present(self.keep + self.logs + self.records)


if __name__ == '__main__':
    unittest.main()
