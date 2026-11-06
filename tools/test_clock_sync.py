"""Synthetic clock truth and malformed transport regressions; no hardware claims."""
import struct
import os
import random
import math
import unittest
from clock_sync import (ANCHOR, Anchor, Affine, ClockMapper, START, TICK, STOP,
                        control, forward32)


def anchor(node, seq, local_sync, local_sample, source, epoch=100, session=77, flags=3):
    return ANCHOR.pack(1, flags, node, 0, session, seq, epoch,
                       round(local_sync), round(local_sample), round(source), 0, 0)


def data(source, epoch=100):
    source = round(source)
    return struct.pack('>H7hII', source & 65535, 1, 2, 3, 4, 5, 6, 7,
                       source & 0xffffffff, epoch)


class SyncTests(unittest.TestCase):
    def train(self, base=1_000_000, count=32):
        mapper = ClockMapper(77)
        # independent oscillator rates and offsets, including sensor-vs-MCU drift
        for seq in range(count):
            common = base + seq * 100_000
            local2 = (common - 8500) / 1.0003
            mapper.anchor(0x711, anchor(1, seq, common, common+500,
                                       (common+500-3200)/1.0002), seq/10)
            mapper.anchor(0x712, anchor(2, seq, local2, local2+500,
                                       (local2+500+1700)/0.9997), seq/10)
        return mapper

    def test_control_and_validation(self):
        self.assertEqual(control(77, 12, START), b'OSYN\x01\x01\x00\x00'+struct.pack('>II',77,12))
        for command in (START, TICK, STOP):
            self.assertEqual(len(control(1, 0xffffffff, command)),16)
        with self.assertRaises(ValueError):
            control(0,0)
        self.assertTrue(forward32(0,0xffffffff))
        self.assertFalse(forward32(3,4))
        with self.assertRaises(ValueError):
            Anchor.decode(0x712,anchor(1,0,1,2,3))
        with self.assertRaises(ValueError):
            Anchor.decode(0x711,anchor(1,0,1,2,3,flags=4))

    def test_held_out_truth(self):
        mapper=self.train()
        # Samples are never included in fitting; compare against generating clocks.
        for j in range(1,50):
            source=4_100_000+j*1250
            expected=(source*0.9997-1700)*1.0003+8500
            sample=mapper.data(2,data(source),3.15+j*0.001)
            self.assertTrue(sample['sync_valid'],sample)
            self.assertLess(abs(sample['common_time_us']-expected),3)

    def test_wrap32_and_64bit_origins(self):
        for base in (2**32-2_000_000,2**48):
            mapper=self.train(base=base)
            source=round((((base+3_150_000)-8500)/1.0003+1700)/0.9997)
            sample=mapper.data(2,data(source),3.2)
            self.assertTrue(sample['sync_valid'],sample)
            self.assertLess(abs(sample['common_time_us']-(base+3_150_000)),3)
            self.assertEqual(sample['source_time_us'],source)

    def test_stale_and_epoch_mismatch(self):
        mapper=self.train()
        self.assertIsNone(mapper.data(2,data(4_200_000,101),3.2)['common_time_us'])
        self.assertIsNone(mapper.data(2,data(4_200_000),4.2)['common_time_us'])
        self.assertIsNone(mapper.data(2,data(4_200_000)[:16],3.2)['common_time_us'])
        self.assertIsNone(mapper.data(2,data(40_000_000),3.2)['common_time_us'])

    def test_reference_reboot_and_old_packets(self):
        mapper=self.train()
        mapper.anchor(0x711,anchor(1,32,1000,1500,500,epoch=101),3.2)
        self.assertFalse(mapper.mcu)
        self.assertIsNone(mapper.data(2,data(4_200_000),3.21)['common_time_us'])
        self.assertFalse(mapper.anchor(0x711,anchor(1,31,4_100_000,4_100_500,4_000_000),3.22))
        self.assertEqual(mapper.reference_epoch,101)

    def test_duplicate_conflict_invalidates(self):
        mapper=self.train()
        original=mapper.groups[31][2]
        packet=anchor(2,31,original.local_sync,original.local_sample,original.source_sample)
        self.assertFalse(mapper.anchor(0x712,packet,3.15))
        self.assertTrue(mapper.mcu)
        corrupt=anchor(2,31,original.local_sync+30,original.local_sample,original.source_sample)
        self.assertFalse(mapper.anchor(0x712,corrupt,3.16))
        self.assertFalse(mapper.mcu)
        self.assertIsNone(mapper.data(2,data(4_200_000),3.17)['common_time_us'])

    def test_explicit_can_invalidation_and_recovery(self):
        mapper=self.train()
        self.assertTrue(mapper.mcu)
        self.assertFalse(mapper.anchor(0x712,anchor(2,31,0,0,0,flags=0),3.15))
        self.assertFalse(mapper.mcu)
        self.assertIn(31,mapper.bad_groups)
        self.assertFalse(mapper.anchor(0x711,anchor(1,31,4_100_000,4_100_500,4_000_000),3.16))
        self.assertIsNone(mapper.data(2,data(4_200_000),3.17)['common_time_us'])
        for seq in range(32,42):
            common=1_000_000+seq*100_000
            local2=(common-8500)/1.0003
            mapper.anchor(0x711,anchor(1,seq,common,common+500,(common+500-3200)/1.0002),seq/10)
            mapper.anchor(0x712,anchor(2,seq,local2,local2+500,(local2+500+1700)/0.9997),seq/10)
        self.assertTrue(mapper.data(2,data(5_200_000),4.2)['sync_valid'])

    def test_sensor_outlier_and_backward_data(self):
        mapper=self.train()
        last=mapper.latest[2]
        mapper.anchor(0x712,anchor(2,32,4_200_000,4_200_500,4_000_000),3.2)
        self.assertEqual(last,mapper.latest[2])
        first=mapper.data(2,data(4_200_000),3.21)
        self.assertTrue(first['sync_valid'])
        second=mapper.data(2,data(4_200_000),3.22)
        self.assertFalse(second['sync_valid'])
        self.assertEqual(second['reason'],'out_of_order_data')

    def test_warmup_outlier_and_quality(self):
        fit=Affine()
        for n in range(12):
            fit.add(n*100000,n*100000+1234+(20000 if n==3 else 0),n/10)
        self.assertTrue(fit.valid(1.15))
        self.assertLess(abs(fit.map(1234567)-1235801),1)
        self.assertFalse(fit.add(1300000,1321234,1.3))
        self.assertFalse(fit.valid(3))

    def test_five_node_independent_clocks(self):
        mapper=ClockMapper(77)
        for k in range(40):
            for node in range(1,6):
                common=1_000_000+k*100000
                local=round((common-node*1000)/(1+(node-1)*0.0001))
                source=round((local+500-node*50)/(1+node*0.00002))
                self.assertTrue(mapper.anchor(0x710+node,
                    anchor(node,k,local,local+500,source,epoch=100+node),k/10))
        self.assertEqual(len(mapper.sensor),5)
        self.assertEqual(len(mapper.mcu),4)
        for node in range(1,6):
            common=4_950_000
            local=(common-node*1000)/(1+(node-1)*0.0001)
            source=(local-node*50)/(1+node*0.00002)
            out=mapper.data(node,data(source,100+node),3.95)
            self.assertTrue(out['sync_valid'],out)
            # Reference node1's local clock is common minus1000, by construction.
            self.assertLess(abs(out['common_time_us']-(common-1000)),3)
        mapper.anchor(0x713,anchor(3,40,1000,1500,500,epoch=999),4)
        self.assertNotIn(3,mapper.mcu)
        self.assertIn(2,mapper.mcu)
        self.assertIsNone(mapper.data(3,data(1000,103),4.01)['common_time_us'])

    def test_reacquire_after_clock_step_and_outage(self):
        fit=Affine()
        for n in range(12):
            fit.add(n*100000,n*100000+1234,n/10)
        self.assertTrue(fit.valid(1.1))
        for n in range(12,32):
            fit.add(n*100000,n*100000+101234,n/10)
        self.assertTrue(fit.valid(3.1))
        self.assertLess(abs(fit.map(3200000)-3301234),1)
        self.assertTrue(fit.add(5000000,5101234,5))
        self.assertFalse(fit.valid(5))
        for n in range(1,9):
            fit.add(5000000+n*100000,5101234+n*100000,5+n/10)
        self.assertTrue(fit.valid(5.8))

    def test_delayed_sample_before_reference_boot_is_invalid(self):
        mapper=self.train()
        for seq in range(32,40):
            t=1_000_000+seq*100000
            mapper.anchor(0x711,anchor(1,seq,t-4_200_000,t-4_200_000+500,
                                      t-4_200_000+500,epoch=101),seq/10)
            mapper.anchor(0x712,anchor(2,seq,t,t+500,t+500),seq/10)
        out=mapper.data(2,data(4_100_000),3.91)
        self.assertFalse(out['sync_valid'])
        self.assertEqual(out['reason'],'before_reference_epoch')
        self.assertEqual(out['source_time_us'],4_100_000)

    def test_retired_epoch_data_cannot_revive_model(self):
        mapper=self.train()
        self.assertTrue(mapper.data(2,data(4_200_000),3.2)['sync_valid'])
        newer=mapper.data(2,data(100,101),3.21)
        self.assertFalse(newer['sync_valid'])
        self.assertNotIn(2,mapper.sensor)
        old=mapper.data(2,data(4_201_250),3.22)
        self.assertEqual(old['reason'],'retired_epoch')
        self.assertFalse(mapper.anchor(0x712,anchor(2,32,4_200_000,4_200_500,4_200_000),3.23))
        self.assertEqual(mapper.epochs[2],101)

    def test_old_epoch_revoke_after_data_barrier_still_revokes_group(self):
        mapper=self.train()
        # A third node shares the potentially ambiguous broadcast.
        mapper.anchor(0x713,anchor(3,31,4_100_000,4_100_500,4_100_500,epoch=103),3.11)
        self.assertIn(3,mapper.mcu)
        mapper.data(2,data(100,101),3.12)
        self.assertNotIn(2,mapper.mcu)
        self.assertIn(3,mapper.mcu)
        # Firmware's delayed revocation identifies the ORIGINAL old epoch.
        mapper.anchor(0x712,anchor(2,31,0,0,0,flags=0),3.13)
        self.assertFalse(mapper.mcu)
        self.assertEqual(mapper.epochs[2],101)
        self.assertIn(31,mapper.bad_groups)

    def test_obsolete_revoke_and_reference_freshness(self):
        mapper=self.train(count=180)
        old_ref=mapper.latest_reference
        # Forgotten old sequence and unannounced old epoch cannot revoke current fits.
        self.assertFalse(mapper.anchor(0x712,anchor(2,0,0,0,0,epoch=99,flags=0),18))
        self.assertTrue(mapper.mcu)
        # Late node arrival completes an old group but must not refresh the reference.
        mapper.anchor(0x713,anchor(3,100,11_000_000,11_000_500,11_000_500),18.1)
        self.assertEqual(mapper.latest_reference,old_ref)

    def test_watermark_survives_same_epoch_revoke(self):
        mapper=self.train()
        first=mapper.data(2,data(4_200_000),3.2)
        self.assertTrue(first['sync_valid'])
        mapper.anchor(0x712,anchor(2,31,0,0,0,flags=0),3.21)
        self.assertEqual(mapper.last_data[2][0],4_200_000)
        for seq in range(32,41):
            common=1_000_000+seq*100_000
            local2=(common-8500)/1.0003
            mapper.anchor(0x711,anchor(1,seq,common,common+500,(common+500-3200)/1.0002),seq/10)
            mapper.anchor(0x712,anchor(2,seq,local2,local2+500,(local2+500+1700)/0.9997),seq/10)
        out=mapper.data(2,data(4_199_000),4.01)
        self.assertFalse(out['sync_valid'])
        self.assertEqual(out['reason'],'out_of_order_data')

    def test_future_and_stale_source_not_guessed(self):
        mapper=self.train()
        base=mapper.latest[2][0].source_sample
        self.assertFalse(mapper.data(2,data(base+500_000),3.11)['sync_valid'])
        self.assertFalse(mapper.data(2,data(base-1_500_000),3.11)['sync_valid'])
        self.assertFalse(mapper.anchor(0x711,anchor(1,32,1,2,3),float('nan')))

    def test_integer_precision_near_uint64_limits(self):
        mapper=ClockMapper(77)
        base=2**63+12345
        for seq in range(12):
            t=base+seq*100000
            mapper.anchor(0x711,anchor(1,seq,t,t+500,t+571),seq/10)
            mapper.anchor(0x712,anchor(2,seq,t+900,t+1400,t+1471),seq/10)
        out=mapper.data(2,data(base+1_150_000),1.15)
        self.assertTrue(out['sync_valid'],out)
        self.assertEqual(out['common_time_us'],base+1_150_000-971)

    def test_poisoned_initial_fit_eventually_recovers(self):
        for bad in (1,2,3,5):
            fit=Affine()
            for seq in range(60):
                fit.add(seq*100000,seq*100000+(100000 if seq<bad else 0),seq/10)
            self.assertTrue(fit.valid(5.9))
            self.assertEqual(fit.map(6_000_000),6_000_000)

    @unittest.skipUnless(os.environ.get('OPENIMU_LONG_SIM') == '1', 'set OPENIMU_LONG_SIM=1 for 2h x5x800Hz simulation')
    def test_two_hour_five_node_randomized_truth(self):
        rng=random.Random(0x43145686)
        mapper=ClockMapper(77)
        boots={n:0 for n in range(1,6)}
        epochs={n:100+n for n in range(1,6)}
        mrate={n:1+(n-3)*0.00005 for n in range(1,6)}
        srate={n:1+(n-2)*0.00012 for n in range(1,6)}
        last_common={}
        valid=invalid=maximum_error=0
        restarts={24000:3,36000:1,60000:4}
        def local(n,t):
            dt=t-boots[n]
            return round(dt*mrate[n]+100*math.sin(dt/1e8+n))+n*1000
        def source(n,t):
            dt=t-boots[n]
            return round(dt*srate[n]+500*math.sin(dt/5e7+n))+n*3000
        for seq in range(72001):
            physical=seq*100000
            if seq in restarts:
                n=restarts[seq]; boots[n]=physical; epochs[n]+=1
            nodes=list(range(1,6)); rng.shuffle(nodes)
            # Random anchor losses/order and identical retransmissions.
            for n in nodes:
                if 7000 <= seq%10000 < 7014 or rng.random()<0.01:
                    continue
                packet=anchor(n,seq,local(n,physical)+rng.choice((-1,0,1)),
                              local(n,physical+1250)+rng.choice((-2,-1,0,1,2)),
                              source(n,physical+1250),epoch=epochs[n])
                mapper.anchor(0x710+n,packet,physical/1e6+0.002)
                if rng.random()<0.01:
                    mapper.anchor(0x710+n,packet,physical/1e6+0.0021)
            # Ambiguous physical retransmission, including revocation loss:
            # the sender changes epoch so subsequent DATA also acts as barrier.
            if seq and seq%997==0:
                n=2; epochs[n]+=1
                if seq%2:
                    mapper.anchor(0x710+n,anchor(n,seq,0,0,0,epoch=epochs[n],flags=0),physical/1e6+0.0022)
            for sample in range(1,81):
                t=physical+sample*1250
                now=t/1e6+0.003
                for n in range(1,6):
                    if rng.random()<0.001:
                        continue
                    packet=data(source(n,t),epochs[n])
                    out=mapper.data(n,packet,now)
                    if out['sync_valid']:
                        error=abs(out['common_time_us']-local(1,t))
                        maximum_error=max(maximum_error,error)
                        self.assertLessEqual(error,15,(seq,n,error,out))
                        key=(n,out['epoch'],out['reference_epoch'])
                        previous=last_common.get(key)
                        if previous is not None:
                            self.assertGreater(out['common_time_us'],previous)
                        last_common[key]=out['common_time_us']
                        valid+=1
                    else:
                        self.assertIsNone(out['common_time_us'])
                        invalid+=1
                    if sample==40 and seq%101==0:
                        duplicate=mapper.data(n,packet,now+0.00001)
                        self.assertFalse(duplicate['sync_valid'])
            if seq and seq%10000==0:
                print(f'simulated_seconds={seq/10:g} valid={valid} invalid={invalid} max_error_us={maximum_error}',flush=True)
            if seq%1000==0:
                self.assertLessEqual(len(mapper.groups),128)
                self.assertLessEqual(len(mapper.bad_groups),128)
                self.assertTrue(all(len(f.points)<=32 for f in (*mapper.sensor.values(),*mapper.mcu.values())))
                self.assertTrue(all(len(v)<=64 for v in mapper.retired_epochs.values()))
        self.assertGreater(valid,27_000_000)
        print(f'2h five-node 800Hz simulation: valid={valid} invalid={invalid} max_error_us={maximum_error}',flush=True)

    def test_unknown_session_and_bad_data(self):
        mapper=self.train()
        self.assertFalse(mapper.anchor(0x711,anchor(1,32,1,2,3,session=78),3.2))
        frame=bytearray(data(4_200_000)); frame[0]^=1
        self.assertEqual(mapper.data(2,frame,3.2)['reason'],'timestamp_mismatch')
        with self.assertRaises(ValueError):
            mapper.data(2,b'\0'*17,3.2)


if __name__=='__main__':
    unittest.main()
