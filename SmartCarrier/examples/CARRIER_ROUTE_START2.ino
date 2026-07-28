/*
 * 启停区2版本入口文件。
 *
 * 实际路线、速度、距离和电机控制代码统一放在CARRIER_ROUTE.ino中，
 * 本文件只在包含公共代码前把CARRIER_START_ZONE设为2。
 *
 * 修改两种启停区共用的路线：
 *   编辑CARRIER_ROUTE.ino中的ROUTE[]。
 *
 * 只修改启停区2的首段/末段：
 *   可在CARRIER_ROUTE.ino中搜索CARRIER_START_ZONE条件编译区域。
 */
#define CARRIER_START_ZONE 2
#include "CARRIER_ROUTE.ino"
