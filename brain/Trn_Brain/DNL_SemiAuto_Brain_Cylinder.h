#define PASSHAND_CLOSE 	GPIO_PIN_RESET
#define PASSHAND_OPEN 	GPIO_PIN_SET
#define PASSARM_UP		GPIO_PIN_RESET
#define PASSARM_DOWN	GPIO_PIN_SET

#define passArm(PASSARM_STATUS) (HAL_GPIO_WritePin(passArm_GPIO_Port, passArm_Pin, PASSARM_STATUS))
#define passHand(PASSHAND_STATUS) (HAL_GPIO_WritePin(passHand_GPIO_Port, passHand_Pin, PASSHAND_STATUS))
