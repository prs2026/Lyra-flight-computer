///*
// * lora.c
// *
// *  Created on: Feb 19, 2026
// *      Author: E
// */
//
//#ifndef INC_LORA_C_
//#define INC_LORA_C_
//
//#include "main.h"
//#include "string.h"
//#include <stdbool.h>
//#include <string.h>
//#include <math.h>
//#include <lora.h>
//
//SUBGHZ_HandleTypeDef hsubghz2;
//
//void LED_on() {
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
//}
//void LED_off() {
//    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
//}
//
//void SetStandbyXOSC() {
//    uint8_t txbuf[2] = {0x80, 0x01};
//    HAL_SUBGHZ_ExecSetCmd(&hsubghz2, txbuf[0], txbuf+1, sizeof(txbuf)-1);
//}
//
//void SetPacketTypeLora() {
//    uint8_t txbuf[2] = {0x8A, 0x01};
//    HAL_SUBGHZ_ExecSetCmd(&hsubghz2, txbuf[0], txbuf+1, sizeof(txbuf)-1);
//}
//
//void SetPacketTypeFSK() {
//    uint8_t txbuf[2] = {0x8A, 0x00};
//    HAL_SUBGHZ_ExecSetCmd(&hsubghz2, txbuf[0], txbuf+1, sizeof(txbuf)-1);
//}
//
//uint32_t ComputeRfFreq(double frequencyMhz) {
//    return (uint32_t)(frequencyMhz * 1048576L); //2^25/(32e6)
//}
//
//void SetRfFreq(uint32_t rfFreq) {
//    uint8_t txbuf[5] = {0x86, (rfFreq & 0xFF000000) >> 24, (rfFreq & 0x00FF0000) >> 16, (rfFreq & 0x0000FF00) >> 8, rfFreq & 0x000000FF};
//    HAL_SUBGHZ_ExecSetCmd(&hsubghz2, txbuf[0], txbuf+1, sizeof(txbuf)-1);
//}
//
//void SetPaLowPower() {
//    // set Pa to 14 dB.
//    uint8_t txbuf[5] = {0x95, 0x02, 0x02, 0x00, 0x01};
//    HAL_SUBGHZ_ExecSetCmd(&hsubghz2, txbuf[0], txbuf+1, sizeof(txbuf)-1);
//}
//
//void SetPa22dB() {
//    // set Pa to the highest 22 dBm
//    uint8_t txbuf[5] = {0x95, 0x04, 0x07, 0x00, 0x01};
//    HAL_SUBGHZ_ExecSetCmd(&hsubghz2, txbuf[0], txbuf+1, sizeof(txbuf)-1);
//}
//
//void SetTxPower(int8_t powerdBm) {
//    // Between -9 and 22
//    int8_t power = powerdBm < -9 ? -9 : ((powerdBm > 22) ? 22 : powerdBm);
//    uint8_t txbuf[3] = {0x8E, (uint8_t) power, 0x02};
//    HAL_SUBGHZ_ExecSetCmd(&hsubghz2, txbuf[0], txbuf+1, sizeof(txbuf)-1);
//}
//
//void SetContinuousWave() {
//    uint8_t txbuf[1] = {0xD1};
//    HAL_SUBGHZ_ExecSetCmd(&hsubghz2, txbuf[0], txbuf, 0);
//}
//
//void SetTxInfinitePreamble() {
//    uint8_t txbuf[1] = {0xD2};
//    HAL_SUBGHZ_ExecSetCmd(&hsubghz2, txbuf[0], txbuf, 0);
//}
//
//void SetTx(uint32_t timeout) {
//    // Timeout * 15.625 µs
//    uint8_t txbuf[4] = {0x83, (timeout & 0x00FF0000) >> 16, (timeout & 0x0000FF00) >> 8, timeout & 0x000000FF};
//    HAL_SUBGHZ_ExecSetCmd(&hsubghz2, txbuf[0], txbuf+1, sizeof(txbuf)-1);
//}
//
//void SetRx(uint32_t timeout) {
//    // Timeout * 15.625 µs
//    // 0x000000 No timeout. Rx Single mode
//    // 0xFFFFFF Rx Continuous mode. The device remains in RX mode until the host sends a command to change the operation mode
//    uint8_t txbuf[4] = {0x82, (timeout & 0x00FF0000) >> 16, (timeout & 0x0000FF00) >> 8, timeout & 0x000000FF};
//    HAL_SUBGHZ_ExecSetCmd(&hsubghz2, txbuf[0], txbuf+1, sizeof(txbuf)-1);
//}
//
//void SetModulationParamsLora(const uint8_t params[4]) {
//    uint8_t txbuf[5] = {0x8B, params[0], params[1], params[2], params[3]};
//    HAL_SUBGHZ_ExecSetCmd(&hsubghz2, txbuf[0], txbuf+1, sizeof(txbuf)-1);
//}
//
//void SetModulationParamsFSK(uint32_t bitrate, uint8_t pulseshape, uint8_t bandwidth, uint32_t freq_dev) {
//    uint32_t BR = 32 * 32e6 / bitrate;
//    uint32_t fdev = (uint32_t) (freq_dev * 1.048576L); // 2^25/32e6 = 1.048576
//    uint8_t txbuf[9] = {0x8B, (BR & 0x00FF0000) >> 16, (BR & 0x0000FF00) >> 8, BR & 0x000000FF, pulseshape, bandwidth, (fdev & 0x00FF0000) >> 16, (fdev & 0x0000FF00) >> 8, fdev & 0x000000FF};
//    HAL_SUBGHZ_ExecSetCmd(&hsubghz2, txbuf[0], txbuf+1, sizeof(txbuf)-1);
//}
//
//void SetPacketParamsLora(uint16_t preamble_length, bool header_fixed, uint8_t payload_length, bool crc_enabled, bool invert_iq) {
//    uint8_t txbuf[7] = {0x8C, (uint8_t)((preamble_length >> 8) & 0xFF), (uint8_t)(preamble_length & 0xFF),
//                        (uint8_t) header_fixed, payload_length, (uint8_t) crc_enabled, (uint8_t) invert_iq};
//
//    HAL_SUBGHZ_ExecSetCmd(&hsubghz2, txbuf[0], txbuf+1, sizeof(txbuf)-1);
//}
///*
//void WriteBuffer(uint8_t offset, uint8_t *data, uint8_t len) {
//    HAL_SUBGHZ_WriteBuffer(&hsubghz, offset, data, len);
//}
//
//void ReadBuffer(uint8_t offset, uint8_t *data, uint8_t len) {
//    HAL_SUBGHZ_ReadBuffer(&hsubghz, offset, data, len);
//}
//*/
//void FSKBeep(int8_t powerdBm, uint32_t toneHz, uint32_t lengthMs) {
//    // assume in standbyXOSC already.
//    HAL_Delay(1);
//    SetTxPower(powerdBm);
//    SetModulationParamsFSK(toneHz*2,    0x09,     0x1E,      2500);
//    HAL_Delay(5);
//    SetTxInfinitePreamble();
//    HAL_Delay(lengthMs);
//    SetStandbyXOSC();
//    HAL_Delay(5);
//}
//
//int CWBeep(int8_t powerdBm, uint32_t lengthMs) {
//    HAL_Delay(1);
//    SetTxPower(powerdBm);
//    HAL_Delay(5);
//    SetContinuousWave();
//    HAL_Delay(lengthMs);
//    SetStandbyXOSC();
//    HAL_Delay(5);
//    return 0;
//}
//
//
//void setupLoRa() {
//	LED_on();
//	HAL_Delay(100);
//	LED_off();
//	SetStandbyXOSC();
//	HAL_Delay(1);
//	SetPacketTypeLora();
//	HAL_Delay(1);
//	SetRfFreq(ComputeRfFreq(915.225));
//
//	//SetPaLowPower(); // For powers up to 14 dBm
//	SetPa22dB(); // Allows powers up to 22 dBm
//	HAL_Delay(1);
//	SetTxPower(22);
//	HAL_Delay(1);
//	//uint8_t LORA_SF12_BW62_CR45[4] = {0x0C, 0x03, 0x01, 0x00};
//	uint8_t LORA_SF8_BW62_CR45[4] = {0x8, 0x03, 0x01, 0x00};
//	SetModulationParamsLora(LORA_SF8_BW62_CR45);
//	HAL_Delay(1);
//
//	SetPacketParamsLora(4, true, 4, true, false); // Send 4 bytes
//	HAL_Delay(1);
//	uint8_t buffer[4] = {0x00, 0x00, 0x00, 0x00};
//	HAL_SUBGHZ_WriteBuffer(&hsubghz2, 0, buffer, 4);
//}
//
//void sendPacket(uint8_t command) {
//	uint8_t buffer[4] = {0x00, 0x00, 0x00, 0x00};
//	buffer[0] = command; // Led ON = 0x04
//	HAL_SUBGHZ_WriteBuffer(&hsubghz2, 0, buffer, 4);
//	HAL_Delay(1);
//	SetTx(0);
//	LED_on();
//	HAL_Delay(1000);
//}
//
//uint8_t recievePacket(){
//	SetRx(0);
//	HAL_Delay(100);
//	uint8_t buffer[4] = {0x00, 0x00, 0x00, 0x00};
//	HAL_SUBGHZ_ReadBuffer(&hsubghz2, 0, buffer, 4);
//	if (buffer[0] == 0x04) {
//	  LED_on();
//	  HAL_Delay(500);
//	  LED_off();
//
//	} else {
//	  LED_off();
//	}
//	return buffer[0];
//}
//
//#endif /* INC_LORA_H_ */
