#ifndef J1939_H_
#define J1939_H_

#define J1939_ADDR_MAIN 0x0C /* MAIN controller                                       */
#define J1939_ADDR_GEND 0x52 /* GENerator of Diesel                                   */
#define J1939_ADDR_GENS 0x66 /* GENerator of Shaft                                    */
#define J1939_ADDR_PSU  0x73 /* Power Supply Unit                                     */
#define J1939_ADDR_SWH0 0x49 /* SWing arm of Horizontal on the front left             */
#define J1939_ADDR_SWH1 0x4A /* SWing arm of Horizontal on the back  left             */
#define J1939_ADDR_SWH2 0x4F /* SWing arm of Horizontal on the back  right            */
#define J1939_ADDR_SWH3 0x4C /* SWing arm of Horizontal on the front right            */
#define J1939_ADDR_RSE0 0x39 /* RaiSE arm on the front left                           */
#define J1939_ADDR_RSE1 0x3A /* RaiSE arm on the back  left                           */
#define J1939_ADDR_RSE2 0x3F /* RaiSE arm on the back  right                          */
#define J1939_ADDR_RSE3 0x3C /* RaiSE arm on the front right                          */
#define J1939_ADDR_SWV0 0x40 /* SWing leg of Vertical on the front left               */
#define J1939_ADDR_SWV1 0x43 /* SWing leg of Vertical on the back  left               */
#define J1939_ADDR_SWV2 0x46 /* SWing leg of Vertical on the back  right              */
#define J1939_ADDR_SWV3 0x45 /* SWing leg of Vertical on the front right              */
#define J1939_ADDR_PRP0 0x30 /* PRoP on the front left                                */
#define J1939_ADDR_PRP1 0x33 /* PRoP on the back  left                                */
#define J1939_ADDR_PRP2 0x36 /* PRoP on the back  right                               */
#define J1939_ADDR_PRP3 0x35 /* PRoP on the front right                               */
#define J1939_ADDR_FX   0x2C /* crane on the Front for X-axis                         */
#define J1939_ADDR_FY0  0x25 /* crane on the Front for Y-axis inside                  */
#define J1939_ADDR_FY1  0x26 /* crane on the Front for Y-axis outside                 */
#define J1939_ADDR_FZ   0x20 /* crane on the Front for Z-axis                         */
#define J1939_ADDR_BX   0x2F /* crane on the Back  for X-axis                         */
#define J1939_ADDR_BY0  0x29 /* crane on the Back  for Y-axis inside                  */
#define J1939_ADDR_BY1  0x2A /* crane on the Back  for Y-axis outside                 */
#define J1939_ADDR_BZ   0x23 /* crane on the Back  for Z-axis                         */
#define J1939_ADDR_LVL0 0x16 /* tilt sensor for LeVeLling                             */
#define J1939_ADDR_LVL1 0x19 /* tilt sensor for LeVeLling as backup                   */
#define J1939_ADDR_VSLF 0x13 /* ViSual Locator on the Front                           */
#define J1939_ADDR_VSLB 0x15 /* ViSual Locator on the Back                            */
#define J1939_ADDR_MOM0 0x83 /* constant MOMent electric machinery on the front left  */
#define J1939_ADDR_MOM1 0x85 /* constant MOMent electric machinery on the back  left  */
#define J1939_ADDR_MOM2 0x89 /* constant MOMent electric machinery on the back  right */
#define J1939_ADDR_MOM3 0x86 /* constant MOMent electric machinery on the front right */
#define J1939_ADDR_TOP  0x93 /* TOP lengthwise electric machinery                     */
#define J1939_ADDR_SDS0 0xA3 /* ShielD of Side on the front left                      */
#define J1939_ADDR_SDS1 0xA5 /* ShielD of Side on the back  left                      */
#define J1939_ADDR_SDS2 0xA9 /* ShielD of Side on the back  right                     */
#define J1939_ADDR_SDS3 0xA6 /* ShielD of Side on the front right                     */
#define J1939_ADDR_SDF0 0xB0 /* ShielD of Front on the far left                       */
#define J1939_ADDR_SDF1 0xB3 /* ShielD of Front on the left                           */
#define J1939_ADDR_SDF2 0xB5 /* ShielD of Front on the right                          */
#define J1939_ADDR_SDF3 0xB6 /* ShielD of Front on the far right                      */
#define J1939_ADDR_SDB0 0xB9 /* ShielD of Back  on the far left                       */
#define J1939_ADDR_SDB1 0xBA /* ShielD of Back  on the left                           */
#define J1939_ADDR_SDB2 0xBC /* ShielD of Back  on the right                          */
#define J1939_ADDR_SDB3 0xBF /* ShielD of Back  on the far right                      */
#define J1939_ADDR_GW   0xFE /* network GateWay                                       */

#define J1939_PRIO_PASSWD     0x08
#define J1939_PRIO_SAVE       0x08
#define J1939_PRIO_QUERY      0x0C
#define J1939_PRIO_FAULT      0x0C
#define J1939_PRIO_SERVO_ZERO 0x08
#define J1939_PRIO_SERVO_CTRL 0x08
#define J1939_PRIO_PSU_CTRL   0x08

#define J1939_FORM_PASSWD     0x40
#define J1939_FORM_SAVE       0x4F
#define J1939_FORM_QUERY      0x5C
#define J1939_FORM_FAULT      0x5F
#define J1939_FORM_SERVO_ZERO 0xA9
#define J1939_FORM_SERVO_AMPR 0xA3
#define J1939_FORM_SERVO_VEL  0xA5
#define J1939_FORM_PSU_CTRL   0xA0

#define J1939_SERVO_ASYNC     0x9A
#define J1939_SERVO_SYNC      0xA9
#define J1939_SERVO_DISABLE   0x3C
#define J1939_SERVO_ENABLE    0xC3

#define J1939_FAULT_NORMAL    0x00
#define J1939_FAULT_WARN      0x03
#define J1939_FAULT_GENERAL   0x0C
#define J1939_FAULT_SERIOUS   0xF0

#endif /* J1939_H_ */
