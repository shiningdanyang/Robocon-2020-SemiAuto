#define PASSHAND_CLOSE 	GPIO_PIN_RESET
#define PASSHAND_OPEN 	GPIO_PIN_SET

#define PASSARM_UP		GPIO_PIN_RESET
#define PASSARM_DOWN	GPIO_PIN_SET

#define GRIPPERQ_CLOSE	GPIO_PIN_RESET
#define GRIPPERQ_OPEN	GPIO_PIN_SET

#define GRIPPERE_CLOSE	GPIO_PIN_RESET
#define GRIPPERE_OPEN	GPIO_PIN_SET

#define PUTQ_UP			GPIO_PIN_SET
#define PUTQ_DOWN		GPIO_PIN_RESET

#define PUTE_UP			GPIO_PIN_SET
#define PUTE_DOWN		GPIO_PIN_RESET


#define passArm(PASSARM_STATUS) (HAL_GPIO_WritePin(passArm_GPIO_Port, passArm_Pin, PASSARM_STATUS))
#define passHand(PASSHAND_STATUS) (HAL_GPIO_WritePin(passHand_GPIO_Port, passHand_Pin, PASSHAND_STATUS))

#define gripperQ(GRIPPERQ_STATUS) (HAL_GPIO_WritePin(gripperQ_GPIO_Port, gripperQ_Pin, GRIPPERQ_STATUS))
#define gripperE(GRIPPERE_STATUS) (HAL_GPIO_WritePin(gripperE_GPIO_Port, gripperE_Pin, GRIPPERE_STATUS))

#define putQ(PUTQ_STATUS) (HAL_GPIO_WritePin(putQ_GPIO_Port, putQ_Pin, PUTQ_STATUS))
#define putE(PUTE_STATUS) (HAL_GPIO_WritePin(putE_GPIO_Port, putE_Pin, PUTE_STATUS))

void cylinder_Init()
{
	gripperE(GRIPPERE_CLOSE);
	gripperQ(GRIPPERQ_CLOSE);
	putQ(PUTQ_UP);
	putE(PUTE_UP);
	passArm(PASSARM_UP);
	passHand(PASSHAND_OPEN);
}
