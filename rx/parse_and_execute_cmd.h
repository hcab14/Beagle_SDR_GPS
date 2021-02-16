#include "kiwi.h"
#include "str.h"
#include "rx_noise.h"

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
                           int& mute, int& de_emp);
