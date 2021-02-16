#include "debug.h"
#include "rx.h"
#include "rx_sound.h"
#include "rx_cmd.h"
#include "agc.h"
#include "spi.h"
#include "fir.h"
#include "fastfir.h"
#include "squelch.h"
#include "dx.h"
#include "wdsp.h"
#include "misc.h"
#include "rx_waterfall.h"
#include "noiseproc.h"
#include "teensy.h"
#include "lms.h"
#include "biquad.h"
#include "parse_and_execute_cmd.h"

//#define TR_SND_CMDS

void parse_and_execute_cmd(conn_t* conn, str_hash_t* snd_cmd_hash, char* cmd,
                           bool& change_freq_mode,
                           u4_t& cmd_recv,
                           bool& masked,
                           bool& change_LPF,
                           int& mode,
                           double& freq,
                           int& chan_null,
                           int& compression,
                           bool& restart,
                           bool& little_endian,
                           int& squelch, int& squelch_on_seq, int& tail_delay,
                           bool& sq_init, bool& squelched,
                           int& nb_algo, int& nr_algo,
                           int* nb_enable, int* nr_enable,
                           float nb_param[NOISE_TYPES][NOISE_PARAMS], float nr_param[NOISE_TYPES][NOISE_PARAMS],
                           int& mute, int& de_emp)
{
    int rx_chan = conn->rx_channel;
    snd_t *snd = &snd_inst[rx_chan];
    double frate = ext_update_get_sample_rateHz(rx_chan);      // FIXME: do this in loop to get incremental changes
    wf_inst_t *wf = &WF_SHMEM->wf_inst[rx_chan];

    TaskStat(TSTAT_INCR|TSTAT_ZERO, 0, "cmd");

    evDP(EC_EVENT, EV_DPUMP, -1, "SND", evprintf("SND: %s", cmd));

#if 0
    if (strcmp(conn->remote_ip, "") == 0 /* && strcmp(cmd, "SET keepalive") != 0 */)
        cprintf(conn, "SND <%s> cmd_recv 0x%x/0x%x\n", cmd, cmd_recv, CMD_ALL);
#endif

    // SECURITY: this must be first for auth check
    if (rx_common_cmd("SND", conn, cmd)) {
        return;
    }

#ifdef TR_SND_CMDS
    if (tr_cmds++ < 32) {
        clprintf(conn, "SND #%02d <%s> cmd_recv 0x%x/0x%x\n", tr_cmds, cmd, cmd_recv, CMD_ALL);
    } else {
        //cprintf(conn, "SND <%s> cmd_recv 0x%x/0x%x\n", cmd, cmd_recv, CMD_ALL);
    }
#endif

    // local state variables
    static int agc = 1;
    static int hang = 0;
    static int thresh = -90;
    static int manGain = 0;
    static int slope = 0;
    static int decay = 50;

    static double locut=0;
    static double hicut=0;

    static double gen = 0;
    static int genattn = 0;

    u2_t key = str_hash_lookup(snd_cmd_hash, cmd);
    bool did_cmd = false;

    switch (key) {

    case CMD_AUDIO_START: {
        int k = 0;
        int n = sscanf(cmd, "SET dbgAudioStart=%d", &k); // value k not used?
        if (n == 1) {
            did_cmd = true;
        }
        break;
    }

    case CMD_TUNE: {
        char *mode_m = NULL;
        double _freq = 0, _locut = 0, _hicut = 0;
        int mparam = 0;
        int n = sscanf(cmd, "SET mod=%16ms low_cut=%lf high_cut=%lf freq=%lf param=%d", &mode_m, &_locut, &_hicut, &_freq, &mparam);
        if ((n == 4 || n == 5) && do_sdr) {
            did_cmd = true;
            //cprintf(conn, "SND f=%.3f lo=%.3f hi=%.3f mode=%s\n", _freq, _locut, _hicut, mode_m);

            bool new_freq = false;
            if (freq != _freq) {
                freq = _freq;
                double f_phase = freq * kHz / conn->adc_clock_corrected;
                u64_t i_phase = (u64_t) round(f_phase * pow(2,48));
                //cprintf(conn, "SND SET freq %.3f kHz i_phase 0x%08x|%08x clk %.3f\n",
                //    freq, PRINTF_U64_ARG(i_phase), conn->adc_clock_corrected);
                if (do_sdr) spi_set3(CmdSetRXFreq, rx_chan, (i_phase >> 16) & 0xffffffff, i_phase & 0xffff);
                cmd_recv |= CMD_FREQ;
                new_freq = true;
                change_freq_mode = true;
            }

            int _mode = kiwi_str2enum(mode_m, mode_s, ARRAY_LEN(mode_s));
            cmd_recv |= CMD_MODE;

            if (_mode == NOT_FOUND) {
                clprintf(conn, "SND bad mode <%s>\n", mode_m);
                _mode = MODE_AM;
                change_freq_mode = true;
            }

            bool new_nbfm = false;
            if (mode != _mode || n == 5) {

                // when switching out of IQ or DRM modes: reset AGC, compression state
                bool IQ_or_DRM_or_SAS = (mode == MODE_IQ || mode == MODE_DRM || mode == MODE_SAS);
                bool new_IQ_or_DRM_or_SAS = (_mode == MODE_IQ || _mode == MODE_DRM || _mode == MODE_SAS);
                if (IQ_or_DRM_or_SAS && !new_IQ_or_DRM_or_SAS && (cmd_recv & CMD_AGC)) {
                    //cprintf(conn, "SND out IQ mode -> reset AGC, compression\n");
                    m_Agc[rx_chan].SetParameters(agc, hang, thresh, manGain, slope, decay, frate);
                    memset(&snd->adpcm_snd, 0, sizeof(ima_adpcm_state_t));
                }

                if (_mode == MODE_SAM && n == 5) {
                    chan_null = mparam;
                }

                // reset SAM demod on non-SAM to SAM transition
                if ((_mode >= MODE_SAM && _mode <= MODE_SAS) && !(mode >= MODE_SAM && mode <= MODE_SAS)) {
                    wdsp_SAM_reset(rx_chan);
                }

                mode = _mode;
                if (mode == MODE_NBFM)
                    new_nbfm = true;
                change_freq_mode = true;
                //cprintf(conn, "SND mode %s\n", mode_m);
            }

            if (mode == MODE_NBFM && (new_freq || new_nbfm)) {
                m_Squelch[rx_chan].Reset();
                conn->last_sample.re = conn->last_sample.im = 0;
            }

            if (hicut != _hicut || locut != _locut) {
                hicut = _hicut; locut = _locut;

                // primary passband filtering
                int fmax = frate/2 - 1;
                if (hicut > fmax) hicut = fmax;
                if (locut < -fmax) locut = -fmax;

                snd->locut = locut; snd->hicut = hicut;

                // normalized passband
                if (locut <= 0 && hicut >= 0) {     // straddles carrier
                    snd->norm_locut = 0.0;
                    snd->norm_hicut = MAX(-locut, hicut);
                } else {
                    if (locut > 0) {
                        snd->norm_locut = locut;
                        snd->norm_hicut = hicut;
                    } else {
                        snd->norm_hicut = -locut;
                        snd->norm_locut = -hicut;
                    }
                }

                // bw for post AM det is max of hi/lo filter cuts
                float bw = fmaxf(fabs(hicut), fabs(locut));
                if (bw > frate/2) bw = frate/2;
                //cprintf(conn, "SND LOcut %.0f HIcut %.0f BW %.0f/%.0f\n", locut, hicut, bw, frate/2);

#define CW_OFFSET 0             // fixme: how is cw offset handled exactly?
                m_PassbandFIR[rx_chan].SetupParameters(0, locut, hicut, CW_OFFSET, frate);
                m_chan_null_FIR[rx_chan].SetupParameters(1, locut, hicut, CW_OFFSET, frate);
                conn->half_bw = bw;

                // post AM detector filter
                // FIXME: not needed if we're doing convolver-based LPF in javascript due to decompression?
                float stop = bw*1.8;
                if (stop > frate/2) stop = frate/2;
                m_AM_FIR[rx_chan].InitLPFilter(0, 1.0, 50.0, bw, stop, frate);
                cmd_recv |= CMD_PASSBAND;

                change_LPF = true;
            }

            double nomfreq = freq;
            if ((hicut-locut) < 1000) nomfreq += (hicut+locut)/2/kHz;   // cw filter correction
            nomfreq = round(nomfreq*kHz);

            conn->freqHz = round(nomfreq/10.0)*10;      // round 10 Hz
            conn->mode = snd->mode = mode;

            // apply masked frequencies
            masked = false;
            if (dx.masked_len != 0 && !conn->tlimit_exempt_by_pwd) {
                int f = round(freq*kHz);
                int pb_lo = f + locut;
                int pb_hi = f + hicut;
                //printf("SND f=%d lo=%.0f|%d hi=%.0f|%d ", f, locut, pb_lo, hicut, pb_hi);
                for (int j=0; j < dx.masked_len; j++) {
                    dx_t *dxp = &dx.list[dx.masked_idx[j]];
                    if (!((pb_hi < dxp->masked_lo || pb_lo > dxp->masked_hi))) {
                        masked = true;
                        //printf("MASKED");
                        break;
                    }
                }
                //printf("\n");
            }
        }
        free(mode_m);
        break;
    }

    case CMD_COMPRESSION: {
        int _comp=0;
        int n = sscanf(cmd, "SET compression=%d", &_comp);
        if (n == 1) {
            did_cmd = true;
            //printf("compression %d\n", _comp);
            if (_comp && (compression != _comp)) {      // when enabling compression reset AGC, compression state
                if (cmd_recv & CMD_AGC)
                    m_Agc[rx_chan].SetParameters(agc, hang, thresh, manGain, slope, decay, frate);
                memset(&snd->adpcm_snd, 0, sizeof(ima_adpcm_state_t));
            }
            compression = _comp;
        }
        break;
    }

    case CMD_REINIT: {
        if (strcmp(cmd, "SET reinit") == 0) {
            did_cmd = true;
            cprintf(conn, "SND restart\n");
            if (cmd_recv & CMD_AGC) {
                m_Agc[rx_chan].SetParameters(agc, hang, thresh, manGain, slope, decay, frate);
            }
            memset(&snd->adpcm_snd, 0, sizeof(ima_adpcm_state_t));
            restart = true;
        }
        break;
    }

    case CMD_LITTLE_ENDIAN: {
        if (strcmp(cmd, "SET little-endian") == 0) {
            did_cmd = true;
            //cprintf(conn, "SND little-endian\n");
            little_endian = true;
        }
        break;
    }

    case CMD_GEN_FREQ: {
        double _gen = 0, mix = 0;
        int n = sscanf(cmd, "SET gen=%lf mix=%lf", &_gen, &mix);
        if (n == 2) {
            did_cmd = true;
            //printf("MIX %f %d\n", mix, (int) mix);
            if (gen != _gen) {
                gen = _gen;
                double f_phase = gen * kHz / conn->adc_clock_corrected;
                u4_t u4_phase = (u4_t) round(f_phase * pow(2,32));
                //printf("sound %d: %s %.3f kHz phase %.3f 0x%08x\n", rx_chan, gen? "GEN_ON":"GEN_OFF", gen, f_phase, u4_phase);
                if (do_sdr) {
                    spi_set(CmdSetGen, 0, u4_phase);
                    ctrl_clr_set(CTRL_USE_GEN, gen? CTRL_USE_GEN:0);
                }
                if (rx_chan == 0) g_genfreq = gen * kHz / ui_srate;
            }
            if (rx_chan == 0) g_mixfreq = mix;
            conn->ext_api = true;
        }
        break;
    }

    case CMD_GEN_ATTN: {
        int _genattn = 0;
        int n = sscanf(cmd, "SET genattn=%d", &_genattn);
        if (n == 1) {
            did_cmd = true;
            if (1 || genattn != _genattn) {
                genattn = _genattn;
                if (do_sdr) spi_set(CmdSetGenAttn, 0, (u4_t) genattn);
                //printf("===> CmdSetGenAttn %d 0x%x\n", genattn, genattn);
                if (rx_chan == 0) g_genampl = genattn / (float)((1<<17)-1);
            }
            conn->ext_api = true;
        }
        break;
    }

    case CMD_SET_AGC: {
        int _agc = 0, _hang = 0, _thresh = 0, _slope = 0, _decay = 0, _manGain = 0;
        int n = sscanf(cmd, "SET agc=%d hang=%d thresh=%d slope=%d decay=%d manGain=%d",
                       &_agc, &_hang, &_thresh, &_slope, &_decay, &_manGain);
        if (n == 6) {
            did_cmd = true;
            agc = _agc;
            hang = _hang;
            thresh = _thresh;
            slope = _slope;
            decay = _decay;
            manGain = _manGain;
            //printf("AGC %d hang=%d thresh=%d slope=%d decay=%d manGain=%d srate=%.1f\n",
            //  agc, hang, thresh, slope, decay, manGain, frate);
            m_Agc[rx_chan].SetParameters(agc, hang, thresh, manGain, slope, decay, frate);
            cmd_recv |= CMD_AGC;
        }
        break;
    }

    case CMD_SQUELCH: {
        int _squelch = 0;
        float _squelch_param = 0;
        int n = sscanf(cmd, "SET squelch=%d param=%f", &_squelch, &_squelch_param);
        if (n == 2) {
            did_cmd = true;
            squelch = _squelch;
            squelched = false;
            //cprintf(conn, "SND SET squelch=%d param=%.2f %s\n", squelch, _squelch_param, mode_s[mode]);
            if (mode == MODE_NBFM) {
                m_Squelch[rx_chan].SetSquelch(squelch, _squelch_param);
            } else {
                float squelch_tail = _squelch_param;
                tail_delay = roundf(squelch_tail * snd_rate / LOOP_BC);
                squelch_on_seq = -1;
                sq_init = true;
            }
        }
        break;
    }

    case CMD_NB_ALGO: {
        int n = sscanf(cmd, "SET nb algo=%d", &nb_algo);
        if (n == 1) {
            did_cmd = true;
            //cprintf(conn, "nb: algo=%d\n", nb_algo);
            memset(nb_enable, 0, NOISE_TYPES*sizeof(int));
            memset(wf->nb_enable, 0, sizeof(wf->nb_enable));
        }
        break;
    }

    case CMD_NR_ALGO: {
        int n = sscanf(cmd, "SET nr algo=%d", &nr_algo);
        if (n == 1) {
            did_cmd = true;
            //cprintf(conn, "nr: algo=%d\n", nr_algo);
            memset(nr_enable, 0, NOISE_TYPES*sizeof(int));
        }
        break;
    }

    case CMD_NB_TYPE: {
        int n_type, n_en, n_param;
        float n_pval;
        if (sscanf(cmd, "SET nb type=%d en=%d", &n_type, &n_en) == 2) {
            did_cmd = true;
            //cprintf(conn, "nb: type=%d en=%d\n", n_type, n_en);
            nb_enable[n_type] = n_en;
            wf->nb_enable[n_type] = n_en;
        } else if (sscanf(cmd, "SET nb type=%d param=%d pval=%f", &n_type, &n_param, &n_pval) == 3) {
            did_cmd = true;
            //cprintf(conn, "nb: type=%d param=%d pval=%.9f\n", n_type, n_param, n_pval);
            nb_param[n_type][n_param] = n_pval;

            if (nb_algo == NB_STD || n_type == NB_CLICK) {
                wf->nb_param[n_type][n_param] = n_pval;
                wf->nb_param_change[n_type] = true;
            }

            if (n_type == NB_BLANKER) {
                switch (nb_algo) {
                case NB_STD: m_NoiseProc[rx_chan][NB_SND].SetupBlanker("SND", frate, nb_param[n_type]); break;
                case NB_WILD: nb_Wild_init(rx_chan, nb_param[n_type]); break;
                }
            }
        }
        break;
    }

    case CMD_NR_TYPE: {
        int n_type, n_en, n_param;
        float n_pval;
        if (sscanf(cmd, "SET nr type=%d en=%d", &n_type, &n_en) == 2) {
            did_cmd = true;
            //cprintf(conn, "nr: type=%d en=%d\n", n_type, n_en);
            nr_enable[n_type] = n_en;
        } else if (sscanf(cmd, "SET nr type=%d param=%d pval=%f", &n_type, &n_param, &n_pval) == 3) {
            did_cmd = true;
            //cprintf(conn, "nr: type=%d param=%d pval=%.9f\n", n_type, n_param, n_pval);
            nr_param[n_type][n_param] = n_pval;

            switch (nr_algo) {
            case NR_WDSP: wdsp_ANR_init(rx_chan, (nr_type_e) n_type, nr_param[n_type]); break;
            case NR_ORIG: m_LMS[rx_chan][n_type].Initialize((nr_type_e) n_type, nr_param[n_type]); break;
            case NR_SPECTRAL: nr_spectral_init(rx_chan, nr_param[n_type]); break;
            }
        }
        break;
    }

    case CMD_MUTE: {
        int n = sscanf(cmd, "SET mute=%d", &mute);
        if (n == 1) {
            did_cmd = true;
            //printf("mute %d\n", mute);
            // FIXME: stop audio stream to save bandwidth?
        }
    }

        // https://dsp.stackexchange.com/questions/34605/biquad-cookbook-formula-for-broadcast-fm-de-emphasis
    case CMD_DE_EMP: {
        int _de_emp;
        int n = sscanf(cmd, "SET de_emp=%d", &_de_emp);
        if (n == 1) {
            did_cmd = true;
            de_emp = _de_emp;
            if (de_emp) {
                TYPEREAL a0, a1, a2, b0, b1, b2;

                // frate 20250 Hz: -20 dB @ 10 kHz
                //  This seems to be the natural filter response when Fs = frate.
                //
                // frate 12000 Hz: -10 dB @  6 kHz
                //  Approximate this by increasing Fs until -10 dB @  6 kHz is achieved
                //  even though this results in an incorrect attenuation curve (too flat).
                double Fs = (snd_rate == SND_RATE_4CH)? frate*6 : frate;
                double T1 = (de_emp == 1)? 0.000075 : 0.000050;
                double z1 = -exp(-1.0/(Fs*T1));
                double p1 = 1.0 + z1;
                a0 = 1.0;
                a1 = p1;
                a2 = 0;
                b0 = 2.0;   // remove filter gain
                b1 = z1;
                b2 = 0;
                m_de_emp_Biquad[rx_chan].InitFilterCoef(a0, a1, a2, b0, b1, b2);
                //cprintf(conn, "SND de-emp: %dus frate %.0f\n", (de_emp == 1)? 75:50, frate);
                //cprintf(conn, "SND de-emp: %f %f %f %f %f %f\n", a0, a1, a2, b0, b1, b2);
            }
        }
        break;
    }

    case CMD_TEST: {
        int test = 0;
        int n = sscanf(cmd, "SET test=%d", &test);
        if (n == 1) {
            did_cmd = true;
            printf("test %d\n", test);
            test_flag = test;
        }
        break;
    }

    case CMD_UAR: {
        int arate_in, arate_out; // not used elsewhere
        int n = sscanf(cmd, "SET UAR in=%d out=%d", &arate_in, &arate_out);
        if (n == 2) {
            did_cmd = true;
            //clprintf(conn, "UAR in=%d out=%d\n", arate_in, arate_out);
        }
        break;
    }

    case CMD_AR_OKAY: {
        int arate_in, arate_out; // not used elsewhere
        int n = sscanf(cmd, "SET AR OK in=%d out=%d", &arate_in, &arate_out);
        if (n == 2) {
            did_cmd = true;
            //clprintf(conn, "AR OK in=%d out=%d\n", arate_in, arate_out);
            if (arate_out) cmd_recv |= CMD_AR_OK;
        }
        break;
    }

    case CMD_UNDERRUN: {
        int j; // not used?
        int n = sscanf(cmd, "SET underrun=%d", &j);
        if (n == 1) {
            did_cmd = true;
            conn->audio_underrun++;
            cprintf(conn, "SND: audio underrun %d %s -------------------------\n",
                    conn->audio_underrun, conn->user);
            //if (ev_dump) evNT(EC_DUMP, EV_NEXTTASK, ev_dump, "NextTask", evprintf("DUMP IN %.3f SEC",
            //  ev_dump/1000.0));
        }
        break;
    }

#ifdef SND_SEQ_CHECK
    case CMD_SEQ: {
        int _seq, _sequence; // not used elsewhere
        int n = sscanf(cmd, "SET seq=%d sequence=%d", &_seq, &_sequence);
        if (n == 2) {
            did_cmd = true;
            conn->sequence_errors++;
            printf("SND%d: audio.js SEQ got %d, expecting %d, %s -------------------------\n",
                   rx_chan, _seq, _sequence, conn->user);
        }
    }
#endif

        // still sent by directTDoA -- ignore
    case CMD_LMS_AUTONOTCH: {
        if (kiwi_str_begins_with(cmd, "SET lms_autonotch")) {
            did_cmd = true;
        }
        break;
    }

    default: {
        // have to check for old API below
        did_cmd = false;
        break;
    }

    } // switch (key)

    if (did_cmd) {
        return;
    }

    // kiwiclient has used "SET nb=" in the past which is shorter than the max_hash_len
    // so must be checked manually
    int nb, th;
    int n = sscanf(cmd, "SET nb=%d th=%d", &nb, &th);
    if (n == 2) {
        nb_param[NB_BLANKER][NB_GATE] = nb;
        nb_param[NB_BLANKER][NB_THRESHOLD] = th;
        nb_enable[NB_BLANKER] = nb? 1:0;

        if (nb) m_NoiseProc[rx_chan][NB_SND].SetupBlanker("SND", frate, nb_param[NB_BLANKER]);
        return;
    }

    if (conn->mc != NULL) {
        cprintf(conn, "#### SND hash=0x%04x key=%d \"%s\"\n", snd_cmd_hash->cur_hash, key, cmd);
        cprintf(conn, "SND BAD PARAMS: sl=%d %d|%d|%d [%s] ip=%s ####################################\n",
                strlen(cmd), cmd[0], cmd[1], cmd[2], cmd, conn->remote_ip);
        conn->unknown_cmd_recvd++;
    }
}
