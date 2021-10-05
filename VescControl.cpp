#include "VescControl.h"
#include "helper.h"
#include "crc.h"
#include "datatypes.h"


#include "mbed_poll.h"
#include "mbed_debug.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
******************************************************************************
VESC UART PROTOCOL
******************************************************************************

******************************************************************************
SMALL PACKET
***************************** LEN 1 byte* ************************************

START_BYTE  PAYLOAD_LENGTH      PAYLOAD         CRC16       END_BYTE

    0x2          0x1              0x4         0x40 0x84        0x3


******************************************************************************
BIG PACKET
***************************** LEN 2 bytes ************************************

START_BYTE  PAYLOAD_LENGTH      PAYLOAD         CRC16       END_BYTE

    0x3       0x01 0xBA           0xE ..      0x40 0x84        0x3

******************************************************************************
*/


VescControl::VescControl(BufferedSerial *pserial) :
m_pSerial(pserial),
m_cmdIdRequested(CMD_NONE)
{
    // set VESC baudrate
    m_pSerial->set_baud(115200);
    // set NON-blocking mode!
    m_pSerial->set_blocking(false);
}

int VescControl::SetRpm(unsigned int rpm)
{
    int payloadIndex = 0;
    // set command id
    m_payloadBuff[payloadIndex++] = CMD_SET_RPM;    
    // set rpm value
    helper_append_int32((char*)m_payloadBuff, rpm, &payloadIndex);
    return send_command(payloadIndex);
}

int VescControl::SetCurrent(float current)
{
    int payloadIndex = 0;
    // set command id
    m_payloadBuff[payloadIndex++] = CMD_SET_CURRENT;
    // set current value
    helper_append_float32((char*)m_payloadBuff, current, 1000.0, &payloadIndex);
    return send_command(payloadIndex);
}

int VescControl::SetCurrentBrake(float current)
{
    int payloadIndex = 0;
    // set command id
    m_payloadBuff[payloadIndex++] = CMD_SET_CURRENT_BRAKE;
    // set current value
    helper_append_float32((char*)m_payloadBuff, current, 1000.0, &payloadIndex);
    return send_command(payloadIndex);
}

int VescControl::SetDutyCycle(float dutyCycle)
{
    int payloadIndex = 0;
    // set command id
    m_payloadBuff[payloadIndex++] = CMD_SET_DUTY;    
    // set current value
    helper_append_float32((char*)m_payloadBuff, dutyCycle, 100000.0, &payloadIndex);
    return send_command(payloadIndex);
}

int VescControl::SetMotorConfig(motor_config_t* pMotorConfig)
{    
	int payloadIndex = 0;
	m_payloadBuff[payloadIndex++] = CMD_SET_MCCONF;

	m_payloadBuff[payloadIndex++] = pMotorConfig->pwm_mode;
	m_payloadBuff[payloadIndex++] = pMotorConfig->comm_mode;
	m_payloadBuff[payloadIndex++] = pMotorConfig->motor_type;
	m_payloadBuff[payloadIndex++] = pMotorConfig->sensor_mode;

	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_current_max, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_current_min, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_in_current_max, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_in_current_min, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_abs_current_max, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_min_erpm, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_max_erpm, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_erpm_start, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_max_erpm_fbrake, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_max_erpm_fbrake_cc, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_min_vin, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_max_vin, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_battery_cut_start, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_battery_cut_end, &payloadIndex);

	m_payloadBuff[payloadIndex++] = pMotorConfig->l_slow_abs_current;

	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_temp_fet_start, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_temp_fet_end, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_temp_motor_start, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_temp_motor_end, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_temp_accel_dec, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_min_duty, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_max_duty, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_watt_max, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->l_watt_min, &payloadIndex);

	helper_append_float32_auto(m_payloadBuff, pMotorConfig->sl_min_erpm, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->sl_min_erpm_cycle_int_limit, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->sl_max_fullbreak_current_dir_change, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->sl_cycle_int_limit, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->sl_phase_advance_at_br, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->sl_cycle_int_rpm_br, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->sl_bemf_coupling_k, &payloadIndex);

	helper_memcpy(m_payloadBuff + payloadIndex, pMotorConfig->hall_table, 8);

	payloadIndex += 8;
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->hall_sl_erpm, &payloadIndex);

	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_current_kp, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_current_ki, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_f_sw, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_dt_us, &payloadIndex);

	m_payloadBuff[payloadIndex++] = pMotorConfig->foc_encoder_inverted;

	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_encoder_offset, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_encoder_ratio, &payloadIndex);

	m_payloadBuff[payloadIndex++] = pMotorConfig->foc_sensor_mode;
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_pll_kp, &payloadIndex);

	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_pll_ki, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_motor_l, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_motor_r, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_motor_flux_linkage, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_observer_gain, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_observer_gain_slow, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_duty_dowmramp_kp, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_duty_dowmramp_ki, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_openloop_rpm, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_sl_openloop_hyst, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_sl_openloop_time, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_sl_d_current_duty, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_sl_d_current_factor, &payloadIndex);

	helper_memcpy(m_payloadBuff + payloadIndex, pMotorConfig->foc_hall_table, 8);

	payloadIndex += 8;
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_sl_erpm, &payloadIndex);

	m_payloadBuff[payloadIndex++] = pMotorConfig->foc_sample_v0_v7;
	m_payloadBuff[payloadIndex++] = pMotorConfig->foc_sample_high_current;

	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_sat_comp, &payloadIndex);

	m_payloadBuff[payloadIndex++] = pMotorConfig->foc_temp_comp;

	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_temp_comp_base_temp, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->foc_current_filter_const, &payloadIndex);

	helper_append_float32_auto(m_payloadBuff, pMotorConfig->s_pid_kp, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->s_pid_ki, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->s_pid_kd, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->s_pid_kd_filter, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->s_pid_min_erpm, &payloadIndex);

	m_payloadBuff[payloadIndex++] = pMotorConfig->s_pid_allow_braking;

	helper_append_float32_auto(m_payloadBuff, pMotorConfig->p_pid_kp, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->p_pid_ki, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->p_pid_kd, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->p_pid_kd_filter, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->p_pid_ang_div, &payloadIndex);

	helper_append_float32_auto(m_payloadBuff, pMotorConfig->cc_startup_boost_duty, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->cc_min_current, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->cc_gain, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->cc_ramp_step_max, &payloadIndex);

	helper_append_int32(m_payloadBuff, pMotorConfig->m_fault_stop_time_ms, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->m_duty_ramp_step, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->m_current_backoff_gain, &payloadIndex);
	helper_append_uint32(m_payloadBuff, pMotorConfig->m_encoder_counts, &payloadIndex);

	m_payloadBuff[payloadIndex++] = pMotorConfig->m_sensor_port_mode;
	m_payloadBuff[payloadIndex++] = pMotorConfig->m_invert_direction;
	m_payloadBuff[payloadIndex++] = pMotorConfig->m_drv8301_oc_mode;
	m_payloadBuff[payloadIndex++] = pMotorConfig->m_drv8301_oc_adj;

	helper_append_float32_auto(m_payloadBuff, pMotorConfig->m_bldc_f_sw_min, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->m_bldc_f_sw_max, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->m_dc_f_sw, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pMotorConfig->m_ntc_motor_beta, &payloadIndex);

	m_payloadBuff[payloadIndex++] = pMotorConfig->m_out_aux_mode;

	return send_command(payloadIndex);
}

int VescControl::SetAppConfig(app_config_t *pAppConfig)
{
    int payloadIndex = 0;
	m_payloadBuff[payloadIndex++] = CMD_GET_APPCONF;

	m_payloadBuff[payloadIndex++] = pAppConfig->controller_id;
	helper_append_uint32(m_payloadBuff, pAppConfig->timeout_msec, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pAppConfig->timeout_brake_current, &payloadIndex);
	m_payloadBuff[payloadIndex++] = pAppConfig->send_can_status;
	helper_append_uint16(m_payloadBuff, pAppConfig->send_can_status_rate_hz, &payloadIndex);
	m_payloadBuff[payloadIndex++] = pAppConfig->can_baud_rate;

	m_payloadBuff[payloadIndex++] = pAppConfig->app_to_use;

	m_payloadBuff[payloadIndex++] = pAppConfig->app_ppm_conf.ctrl_type;
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_ppm_conf.pid_max_erpm, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_ppm_conf.hyst, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_ppm_conf.pulse_start, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_ppm_conf.pulse_end, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_ppm_conf.pulse_center, &payloadIndex);
	m_payloadBuff[payloadIndex++] = pAppConfig->app_ppm_conf.median_filter;
	m_payloadBuff[payloadIndex++] = pAppConfig->app_ppm_conf.safe_start;
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_ppm_conf.throttle_exp, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_ppm_conf.throttle_exp_brake, &payloadIndex);
	m_payloadBuff[payloadIndex++] = pAppConfig->app_ppm_conf.throttle_exp_mode;
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_ppm_conf.ramp_time_pos, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_ppm_conf.ramp_time_neg, &payloadIndex);
	m_payloadBuff[payloadIndex++] = pAppConfig->app_ppm_conf.multi_esc;
	m_payloadBuff[payloadIndex++] = pAppConfig->app_ppm_conf.tc;
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_ppm_conf.tc_max_diff, &payloadIndex);

	m_payloadBuff[payloadIndex++] = pAppConfig->app_adc_conf.ctrl_type;
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_adc_conf.hyst, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_adc_conf.voltage_start, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_adc_conf.voltage_end, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_adc_conf.voltage_center, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_adc_conf.voltage2_start, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_adc_conf.voltage2_end, &payloadIndex);
	m_payloadBuff[payloadIndex++] = pAppConfig->app_adc_conf.use_filter;
	m_payloadBuff[payloadIndex++] = pAppConfig->app_adc_conf.safe_start;
	m_payloadBuff[payloadIndex++] = pAppConfig->app_adc_conf.cc_button_inverted;
	m_payloadBuff[payloadIndex++] = pAppConfig->app_adc_conf.rev_button_inverted;
	m_payloadBuff[payloadIndex++] = pAppConfig->app_adc_conf.voltage_inverted;
	m_payloadBuff[payloadIndex++] = pAppConfig->app_adc_conf.voltage2_inverted;
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_adc_conf.throttle_exp, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_adc_conf.throttle_exp_brake, &payloadIndex);
	m_payloadBuff[payloadIndex++] = pAppConfig->app_adc_conf.throttle_exp_mode;
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_adc_conf.ramp_time_pos, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_adc_conf.ramp_time_neg, &payloadIndex);
	m_payloadBuff[payloadIndex++] = pAppConfig->app_adc_conf.multi_esc;
	m_payloadBuff[payloadIndex++] = pAppConfig->app_adc_conf.tc;
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_adc_conf.tc_max_diff, &payloadIndex);
	helper_append_uint16(m_payloadBuff, pAppConfig->app_adc_conf.update_rate_hz, &payloadIndex);

	helper_append_uint32(m_payloadBuff, pAppConfig->app_uart_baudrate, &payloadIndex);

	m_payloadBuff[payloadIndex++] = pAppConfig->app_chuk_conf.ctrl_type;
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_chuk_conf.hyst, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_chuk_conf.ramp_time_pos, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_chuk_conf.ramp_time_neg, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_chuk_conf.stick_erpm_per_s_in_cc, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_chuk_conf.throttle_exp, &payloadIndex);
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_chuk_conf.throttle_exp_brake, &payloadIndex);
	m_payloadBuff[payloadIndex++] = pAppConfig->app_chuk_conf.throttle_exp_mode;
	m_payloadBuff[payloadIndex++] = pAppConfig->app_chuk_conf.multi_esc;
	m_payloadBuff[payloadIndex++] = pAppConfig->app_chuk_conf.tc;
	helper_append_float32_auto(m_payloadBuff, pAppConfig->app_chuk_conf.tc_max_diff, &payloadIndex);

	m_payloadBuff[payloadIndex++] = pAppConfig->app_nrf_conf.speed;
	m_payloadBuff[payloadIndex++] = pAppConfig->app_nrf_conf.power;
	m_payloadBuff[payloadIndex++] = pAppConfig->app_nrf_conf.crc_type;
	m_payloadBuff[payloadIndex++] = pAppConfig->app_nrf_conf.retry_delay;
	m_payloadBuff[payloadIndex++] = pAppConfig->app_nrf_conf.retries;
	m_payloadBuff[payloadIndex++] = pAppConfig->app_nrf_conf.channel;

	helper_memcpy(m_payloadBuff + payloadIndex, pAppConfig->app_nrf_conf.address, 3);
    
	payloadIndex += 3;
	m_payloadBuff[payloadIndex++] = pAppConfig->app_nrf_conf.send_crc_ack;

    return send_command(payloadIndex);
}

void VescControl::GetMotorValues()
{
   int payloadIndex = 0;
    // set command id
    m_payloadBuff[payloadIndex++] = CMD_GET_VALUES;
    // set requested id of the receive function
    m_cmdIdRequested = CMD_GET_VALUES;
    send_command(payloadIndex);  
}

void VescControl::GetFirmwareVersion()
{
    int payloadIndex = 0;
    // set command id
    m_payloadBuff[payloadIndex++] = CMD_FW_VERSION;
    // set requested id of the receive function
    m_cmdIdRequested = CMD_FW_VERSION;
    send_command(payloadIndex);
}

void VescControl::GetMotorConfig(bool bReadDefaultConfig)
{
    int payloadIndex = 0;
    // set command id
    m_payloadBuff[payloadIndex++] = bReadDefaultConfig ? CMD_GET_MCCONF_DEFAULT : CMD_GET_MCCONF;
    // set requested id of the receive function
    m_cmdIdRequested = CMD_GET_MCCONF;
    send_command(payloadIndex);
}
    
void VescControl::GetAppConfig(bool bReadDefaultConfig)
{
    int payloadIndex = 0;
    // set command id
    m_payloadBuff[payloadIndex++] = bReadDefaultConfig ? CMD_GET_APPCONF_DEFAULT : CMD_GET_APPCONF;
    // set requested id of the receive function
    m_cmdIdRequested = CMD_GET_APPCONF;
    send_command(payloadIndex);
}

void VescControl::RebootController()
{
    int payloadIndex = 0;
    // set command id
    m_payloadBuff[payloadIndex++] = CMD_REBOOT;
    send_command(payloadIndex);
}

void VescControl::SendAlive()
{
    int payloadIndex = 0;
    // set command id
    m_payloadBuff[payloadIndex++] = CMD_ALIVE;
    send_command(payloadIndex);
}

bool VescControl::Read()
{
    unsigned short crc_read;
    unsigned int bytes_read = 0;

    // check requested id
    if( m_cmdIdRequested == CMD_GET_VALUES )
    {
        if( (bytes_read = m_pSerial->read(m_rxData.rxBuff, PACKET_LENGTH_MOTOR_VALUES)) == PACKET_LENGTH_MOTOR_VALUES )
        {
            // check start and end byte
            if( (m_rxData.rxBuff[0] == VESC_START_BYTE_SMALL) && (m_rxData.rxBuff[PACKET_LENGTH_MOTOR_VALUES-1] == VESC_END_BYTE) )
            {
                m_rxData.payload_length = m_rxData.rxBuff[1];

                m_rxData.crc = crc16(&m_rxData.rxBuff[PLOAD_INDEX_SMALL_PACKET], PLOAD_LENGTH_MOTOR_VALUES);

                crc_read = (m_rxData.rxBuff[PACKET_LENGTH_MOTOR_VALUES-CRCH_TAIL_OFFSET] << 8) | m_rxData.rxBuff[PACKET_LENGTH_MOTOR_VALUES-CRCL_TAIL_OFFSET];

                // check CRC and payload length
                if( (m_rxData.crc == crc_read) && (m_rxData.payload_length == PLOAD_LENGTH_MOTOR_VALUES) )
                {
                    // read values and reset requested command
                    read_motor_values(&m_rxData.rxBuff[PLOAD_INDEX_SMALL_PACKET+1], m_rxData.payload_length);
                    m_cmdIdRequested = CMD_NONE;
                    return true;
                }
            }
        }
        return false;
    }
    else if( m_cmdIdRequested == CMD_GET_MCCONF || m_cmdIdRequested == CMD_GET_MCCONF_DEFAULT )
    {
        if( (bytes_read = m_pSerial->read(m_rxData.rxBuff, PACKET_LENGTH_MOTOR_CONFIG)) == PACKET_LENGTH_MOTOR_CONFIG )
        {
            // check start and end byte
            if( (m_rxData.rxBuff[0] == VESC_START_BYTE_BIG) && (m_rxData.rxBuff[PACKET_LENGTH_MOTOR_CONFIG-1] == VESC_END_BYTE) )
            {
                m_rxData.payload_length = m_rxData.rxBuff[1] << 8 | m_rxData.rxBuff[2];

                m_rxData.crc = crc16(&m_rxData.rxBuff[PLOAD_INDEX_BIG_PACKET], PLOAD_LENGTH_MOTOR_CONFIG);
                crc_read = (m_rxData.rxBuff[PACKET_LENGTH_MOTOR_CONFIG-CRCH_TAIL_OFFSET] << 8) | m_rxData.rxBuff[PACKET_LENGTH_MOTOR_CONFIG-CRCL_TAIL_OFFSET];

                // check CRC and payload length
                if( (m_rxData.crc == crc_read) && (m_rxData.payload_length == PLOAD_LENGTH_MOTOR_CONFIG) )
                {
                    // read values and reset requested command
                    read_motor_config(&m_rxData.rxBuff[PLOAD_INDEX_BIG_PACKET+1], m_rxData.payload_length, m_cmdIdRequested == CMD_GET_MCCONF_DEFAULT );
                    return true;
                }
            }
        }
        return false;
    }
    else if( m_cmdIdRequested == CMD_GET_APPCONF || m_cmdIdRequested == CMD_GET_APPCONF_DEFAULT )
    {
        if( (bytes_read = m_pSerial->read(m_rxData.rxBuff, PACKET_LENGTH_APP_CONFIG)) == PACKET_LENGTH_APP_CONFIG )
        {
            // check start and end byte
            if( (m_rxData.rxBuff[0] == VESC_START_BYTE_BIG) && (m_rxData.rxBuff[PACKET_LENGTH_APP_CONFIG-1] == VESC_END_BYTE) )
            {
                m_rxData.payload_length = m_rxData.rxBuff[1] << 8 | m_rxData.rxBuff[2];

                m_rxData.crc = crc16(&m_rxData.rxBuff[PLOAD_INDEX_BIG_PACKET], PLOAD_LENGTH_APP_CONFIG);
                crc_read = (m_rxData.rxBuff[PACKET_LENGTH_APP_CONFIG-CRCH_TAIL_OFFSET] << 8) | m_rxData.rxBuff[PACKET_LENGTH_APP_CONFIG-CRCL_TAIL_OFFSET];

                // check CRC and payload length
                if( (m_rxData.crc == crc_read) && (m_rxData.payload_length == PLOAD_LENGTH_APP_CONFIG) )
                {
                    // read values and reset requested command
                    read_app_config(&m_rxData.rxBuff[PLOAD_INDEX_BIG_PACKET+1], m_rxData.payload_length, m_cmdIdRequested == CMD_GET_APPCONF_DEFAULT);
                    return true;
                }
            }
        }
        return false;
    }
    else if( m_cmdIdRequested == CMD_FW_VERSION )
    {
        return false;
    }
    else
    {
        // nothing requested -> nothing to read
        return false;
    }
}

int VescControl::send_command(unsigned int payload_length)
{
    int txIndex = 1;
    unsigned short crc_value = 0;

    // add start-byte to tx buffer
    m_txBuff[0] = VESC_START_BYTE_SMALL;    

    // add payload length to tx buffer
    m_txBuff[txIndex++] = payload_length;

    // add payload to tx buffer
    for(int i = 0; i < payload_length; i++)
    {
        m_txBuff[txIndex] = m_payloadBuff[i];
        txIndex++;
    }

    // compute and add crc values to tx buffer
    crc_value = crc16(&m_txBuff[2], payload_length);
    m_txBuff[txIndex++] = crc_value >> 8;
    m_txBuff[txIndex++] = crc_value & 0xFF;

    // add end-byte to tx buffer
    m_txBuff[txIndex++] = VESC_END_BYTE;

    // send tx buffer and return written bytes
    return m_pSerial->write(m_txBuff, txIndex);
}


void VescControl::read_motor_values(const char* payload, size_t length)
{
    int payloadIndex = 0;

    if( payload == NULL )
    {
        return;
    }   
    
    m_motorValues.temp_mos = helper_get_float16(payload, 1e1, &payloadIndex);
    m_motorValues.temp_motor = helper_get_float16(payload, 1e1, &payloadIndex);
    m_motorValues.current_motor = helper_get_float32(payload, 1e2, &payloadIndex);
    m_motorValues.current_in = helper_get_float32(payload, 1e2, &payloadIndex);
    m_motorValues.id = helper_get_float32(payload, 1e2, &payloadIndex);
    m_motorValues.iq = helper_get_float32(payload, 1e2, &payloadIndex);
    m_motorValues.duty_now = helper_get_float16(payload, 1e3, &payloadIndex);
    m_motorValues.rpm = helper_get_float32(payload, 1e0, &payloadIndex);
    m_motorValues.v_in = helper_get_float16(payload, 1e1, &payloadIndex);
    m_motorValues.amp_hours = helper_get_float32(payload, 1e4, &payloadIndex);
    m_motorValues.amp_hours_charged = helper_get_float32(payload, 1e4, &payloadIndex);
    m_motorValues.watt_hours = helper_get_float32(payload, 1e4, &payloadIndex);
    m_motorValues.watt_hours_charged = helper_get_float32(payload, 1e4, &payloadIndex);
    m_motorValues.tachometer = helper_get_int32(payload, &payloadIndex);
    m_motorValues.tachometer_abs = helper_get_int32(payload, &payloadIndex);
    m_motorValues.fault_code = (mc_fault_code)payload[payloadIndex++];
    m_motorValues.pid_pos = (payloadIndex < (int)length) ? helper_get_float32(payload, 1e6, &payloadIndex) : 0.0;
    m_motorValues.vesc_id = (payloadIndex < (int)length) ? payload[payloadIndex++] : 255;
}

void VescControl::read_motor_config(const char* payload, size_t length, bool bIsDefaultConfig)
{
    int payloadIndex = 0;

    motor_config_t* pMotorConfig = bIsDefaultConfig ? &m_motorConfig : &m_motorConfigDefault;

    if( payload == NULL )
    {
        return;
    }   

    pMotorConfig->pwm_mode = (mc_pwm_mode) payload[payloadIndex++];
    pMotorConfig->comm_mode = (mc_comm_mode) payload[payloadIndex++];
    pMotorConfig->motor_type = (mc_motor_type) payload[payloadIndex++];
    pMotorConfig->sensor_mode = (mc_sensor_mode) payload[payloadIndex++];

    pMotorConfig->l_current_max = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_current_min = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_in_current_max = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_in_current_min = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_abs_current_max = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_min_erpm = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_max_erpm = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_erpm_start = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_max_erpm_fbrake = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_max_erpm_fbrake_cc = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_min_vin = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_max_vin = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_battery_cut_start = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_battery_cut_end = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_slow_abs_current = payload[payloadIndex++];
    pMotorConfig->l_temp_fet_start = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_temp_fet_end = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_temp_motor_start = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_temp_motor_end = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_temp_accel_dec = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_min_duty = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_max_duty = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_watt_max = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->l_watt_min = helper_get_float32_auto(payload, &payloadIndex);

    pMotorConfig->lo_current_max = pMotorConfig->l_current_max;
    pMotorConfig->lo_current_min = pMotorConfig->l_current_min;
    pMotorConfig->lo_in_current_max = pMotorConfig->l_in_current_max;
    pMotorConfig->lo_in_current_min = pMotorConfig->l_in_current_min;
    pMotorConfig->lo_current_motor_max_now = pMotorConfig->l_current_max;
    pMotorConfig->lo_current_motor_min_now = pMotorConfig->l_current_min;

    pMotorConfig->sl_min_erpm = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->sl_min_erpm_cycle_int_limit = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->sl_max_fullbreak_current_dir_change = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->sl_cycle_int_limit = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->sl_phase_advance_at_br = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->sl_cycle_int_rpm_br = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->sl_bemf_coupling_k = helper_get_float32_auto(payload, &payloadIndex);

    helper_memcpy(pMotorConfig->hall_table, payload + payloadIndex, 8);

    payloadIndex += 8;
    pMotorConfig->hall_sl_erpm = helper_get_float32_auto(payload, &payloadIndex);

    pMotorConfig->foc_current_kp = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_current_ki = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_f_sw = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_dt_us = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_encoder_inverted = payload[payloadIndex++];
    pMotorConfig->foc_encoder_offset = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_encoder_ratio = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_sensor_mode = (mc_foc_sensor_mode) payload[payloadIndex++];
    pMotorConfig->foc_pll_kp = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_pll_ki = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_motor_l = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_motor_r = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_motor_flux_linkage = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_observer_gain = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_observer_gain_slow = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_duty_dowmramp_kp = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_duty_dowmramp_ki = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_openloop_rpm = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_sl_openloop_hyst = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_sl_openloop_time = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_sl_d_current_duty = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_sl_d_current_factor = helper_get_float32_auto(payload, &payloadIndex);
    
    helper_memcpy(pMotorConfig->foc_hall_table, payload + payloadIndex, 8);

    payloadIndex += 8;
    pMotorConfig->foc_sl_erpm = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_sample_v0_v7 = payload[payloadIndex++];
    pMotorConfig->foc_sample_high_current = payload[payloadIndex++];
    pMotorConfig->foc_sat_comp = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_temp_comp = payload[payloadIndex++];
    pMotorConfig->foc_temp_comp_base_temp = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->foc_current_filter_const = helper_get_float32_auto(payload, &payloadIndex);

    pMotorConfig->s_pid_kp = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->s_pid_ki = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->s_pid_kd = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->s_pid_kd_filter = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->s_pid_min_erpm = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->s_pid_allow_braking = payload[payloadIndex++];

    pMotorConfig->p_pid_kp = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->p_pid_ki = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->p_pid_kd = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->p_pid_kd_filter = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->p_pid_ang_div = helper_get_float32_auto(payload, &payloadIndex);

    pMotorConfig->cc_startup_boost_duty = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->cc_min_current = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->cc_gain = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->cc_ramp_step_max = helper_get_float32_auto(payload, &payloadIndex);

    pMotorConfig->m_fault_stop_time_ms = helper_get_int32(payload, &payloadIndex);
    pMotorConfig->m_duty_ramp_step = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->m_current_backoff_gain = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->m_encoder_counts = helper_get_uint32(payload, &payloadIndex);
    pMotorConfig->m_sensor_port_mode = (sensor_port_mode) payload[payloadIndex++];
    pMotorConfig->m_invert_direction = payload[payloadIndex++];
    pMotorConfig->m_drv8301_oc_mode = (drv8301_oc_mode) payload[payloadIndex++];
    pMotorConfig->m_drv8301_oc_adj = payload[payloadIndex++];
    pMotorConfig->m_bldc_f_sw_min = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->m_bldc_f_sw_max = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->m_dc_f_sw = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->m_ntc_motor_beta = helper_get_float32_auto(payload, &payloadIndex);
    pMotorConfig->m_out_aux_mode = (out_aux_mode) payload[payloadIndex++];
}

void VescControl::read_app_config(const char* payload, size_t length, bool bIsDefaultConfig)
{
    int payloadIndex = 0;

    app_config_t* pAppConfig = bIsDefaultConfig ? &m_appConfigDefault : &m_appConfig;

    if( payload == NULL )
    {
        return;
    }   

    pAppConfig->controller_id = payload[payloadIndex++];
    pAppConfig->timeout_msec = helper_get_uint32(payload, &payloadIndex);
    pAppConfig->timeout_brake_current = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->send_can_status = payload[payloadIndex++];
    pAppConfig->send_can_status_rate_hz = helper_get_uint16(payload, &payloadIndex);
    pAppConfig->can_baud_rate = (CAN_BAUD) payload[payloadIndex++];

    pAppConfig->app_to_use = (app_use) payload[payloadIndex++];

    pAppConfig->app_ppm_conf.ctrl_type = (ppm_control_type) payload[payloadIndex++];
    pAppConfig->app_ppm_conf.pid_max_erpm = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_ppm_conf.hyst = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_ppm_conf.pulse_start = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_ppm_conf.pulse_end = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_ppm_conf.pulse_center = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_ppm_conf.median_filter = payload[payloadIndex++];
    pAppConfig->app_ppm_conf.safe_start = payload[payloadIndex++];
    pAppConfig->app_ppm_conf.throttle_exp = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_ppm_conf.throttle_exp_brake = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_ppm_conf.throttle_exp_mode = (thr_exp_mode) payload[payloadIndex++];
    pAppConfig->app_ppm_conf.ramp_time_pos = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_ppm_conf.ramp_time_neg = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_ppm_conf.multi_esc = payload[payloadIndex++];
    pAppConfig->app_ppm_conf.tc = payload[payloadIndex++];
    pAppConfig->app_ppm_conf.tc_max_diff = helper_get_float32_auto(payload, &payloadIndex);

    pAppConfig->app_adc_conf.ctrl_type = (adc_control_type) payload[payloadIndex++];
    pAppConfig->app_adc_conf.hyst = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_adc_conf.voltage_start = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_adc_conf.voltage_end = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_adc_conf.voltage_center = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_adc_conf.voltage2_start = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_adc_conf.voltage2_end = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_adc_conf.use_filter = payload[payloadIndex++];
    pAppConfig->app_adc_conf.safe_start = payload[payloadIndex++];
    pAppConfig->app_adc_conf.cc_button_inverted = payload[payloadIndex++];
    pAppConfig->app_adc_conf.rev_button_inverted = payload[payloadIndex++];
    pAppConfig->app_adc_conf.voltage_inverted = payload[payloadIndex++];
    pAppConfig->app_adc_conf.voltage2_inverted = payload[payloadIndex++];
    pAppConfig->app_adc_conf.throttle_exp = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_adc_conf.throttle_exp_brake = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_adc_conf.throttle_exp_mode = (thr_exp_mode) payload[payloadIndex++];
    pAppConfig->app_adc_conf.ramp_time_pos = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_adc_conf.ramp_time_neg = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_adc_conf.multi_esc = payload[payloadIndex++];
    pAppConfig->app_adc_conf.tc = payload[payloadIndex++];
    pAppConfig->app_adc_conf.tc_max_diff = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_adc_conf.update_rate_hz = helper_get_uint16(payload, &payloadIndex);

    pAppConfig->app_uart_baudrate = helper_get_uint32(payload, &payloadIndex);

    pAppConfig->app_chuk_conf.ctrl_type = (chuk_control_type) payload[payloadIndex++];
    pAppConfig->app_chuk_conf.hyst = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_chuk_conf.ramp_time_pos = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_chuk_conf.ramp_time_neg = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_chuk_conf.stick_erpm_per_s_in_cc = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_chuk_conf.throttle_exp = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_chuk_conf.throttle_exp_brake = helper_get_float32_auto(payload, &payloadIndex);
    pAppConfig->app_chuk_conf.throttle_exp_mode = (thr_exp_mode) payload[payloadIndex++];
    pAppConfig->app_chuk_conf.multi_esc = payload[payloadIndex++];
    pAppConfig->app_chuk_conf.tc = payload[payloadIndex++];
    pAppConfig->app_chuk_conf.tc_max_diff = helper_get_float32_auto(payload, &payloadIndex);

    pAppConfig->app_nrf_conf.speed = (NRF_SPEED) payload[payloadIndex++];
    pAppConfig->app_nrf_conf.power = (NRF_POWER) payload[payloadIndex++];
    pAppConfig->app_nrf_conf.crc_type = (NRF_CRC) payload[payloadIndex++];
    pAppConfig->app_nrf_conf.retry_delay = (NRF_RETR_DELAY) payload[payloadIndex++];
    pAppConfig->app_nrf_conf.retries = payload[payloadIndex++];
    pAppConfig->app_nrf_conf.channel = payload[payloadIndex++];

    helper_memcpy(pAppConfig->app_nrf_conf.address, payload + payloadIndex, 3);

    payloadIndex += 3;
    pAppConfig->app_nrf_conf.send_crc_ack = payload[payloadIndex++];
}