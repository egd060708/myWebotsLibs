#pragma once
#include <iostream>
#include "myMatrices.h"
#include "math.h"
#include "Upper_Public.h"

using namespace mM;//����������ռ�

enum class rOrder {
	XYZ,
	XZY,
	YXZ,
	YZX,
	ZXY,
	ZYX
};

const float zero_check = 0.001f;

template<uint8_t jNum>
class Arm_Def_s
{
public:
	myMatrices<float> DH_params = myMatrices<float>(jNum, 5);//DH�����洢����(4����������+1���ؽ�offset)
	myMatrices<float> jointSpace_t = myMatrices<float>(1, jNum);//Ŀ��ؽڿռ�(����������ˣ���ȥoffset)
	myMatrices<float> jointSpace_c = myMatrices<float>(1, jNum);//��ǰ�ؽڿռ�(�ṩ���������㣬����offset)
	myMatrices<float> workSpace_t = myMatrices<float>(4) ;//Ŀ�깤���ռ�(��������ϵ��λ�ˣ�
	myMatrices<float> workSpace_c = myMatrices<float>(4) ;//��ǰ�����ռ�
	myMatrices<float> workArray_t = myMatrices<float>(1, 6);//�����ռ�������ʽ
	myMatrices<float> workArray_c = myMatrices<float>(1, 6);
	myMatrices<float> spaceCali = myMatrices<float>(4);//ĩ����̬У׼����
	myMatrices<float> jointCs = myMatrices<float>(jNum, 2);//�ؽ�Ŀ��Լ��
	myMatrices<float> workArrayCs = myMatrices<float>(6, 2);//Ŀ������Լ��
	myMatrices<float> fk();//���˶�ѧ����
	myMatrices<float> ik();//���˶�ѧ����
	myMatrices<float> T2A(myMatrices<float> _T);//��α任����ת����ά����
	rOrder order_c;
//public:
	Arm_Def_s(){}
	void set_eularOrder(rOrder _order) { order_c = _order; };
	void import_DH(uint8_t _num, float _alpha, float _a, float _d, float _theta, float _offset);//����DH����
	void update_Space_c(const float _angle[jNum]);//���µ�ǰ�ռ�
	void update_Space_t(const float _endSpace[6]);//����Ŀ��ռ�(ĩ�˲�����ά����+ŷ����������ʾ��
	void update_Space_t(myMatrices<float> _T);//����Ŀ��ռ�(����ת�����ʾ��
	void set_spaceCali(const float _spaceCali[4 * 4]);//������̬У׼����
	void set_jointCs(const float _acs[2 * jNum]);//���ùؽڿռ�Ŀ��Լ��
	void set_workArrayCs(const float _wcs[2 * 6]);//���ÿռ�����Ŀ��Լ��
};

template<uint8_t jNum>
void Arm_Def_s<jNum>::import_DH(uint8_t _num, float _alpha, float _a, float _d, float _theta, float _offset)
{
	if (_num > 0)
	{
		float importValue[5] = { _alpha,_a,_d,_theta,_offset };
		DH_params.setRowArray(importValue, _num - 1);
	}
}

template<uint8_t jNum>
void Arm_Def_s<jNum>::update_Space_c(const float _angle[jNum])
{
	//�����ؽڿռ�ֵ,������DH��������
	for (int i = 0; i < jNum; i++)
	{
		jointSpace_c.setElement(0,i, _angle[i] + DH_params.getElement(i, 4));
		DH_params.setElement(i, 3, jointSpace_c.getElement(0,i));
	}
	//���˶�ѧ����
	workSpace_c = fk();
}

template<uint8_t jNum>
void Arm_Def_s<jNum>::update_Space_t(const float _endSpace[6])
{
	workArray_t.setRowArray(_endSpace, 0);
	float T1_array[4 * 4] = { 1,				 0,					 0,	0,
							  0,cosf(_endSpace[3]),-sinf(_endSpace[3]), 0,
							  0,sinf(_endSpace[3]), cosf(_endSpace[3]), 0,
							  0,				 0,					 0, 1 };
	float T2_array[4 * 4] = {  cosf(_endSpace[4]), 0,sinf(_endSpace[4]),0,
												0, 1,				  0,0,
							  -sinf(_endSpace[4]), 0,cosf(_endSpace[4]),0,
												0, 0,				  0,1 };
	float T3_array[4 * 4] = { cosf(_endSpace[5]),-sinf(_endSpace[5]),0, 0,
							  sinf(_endSpace[5]), cosf(_endSpace[5]),0, 0,
											   0,				   0,1, 0,
											   0,			       0,0, 1 };
	myMatrices<float> T1(4);
	myMatrices<float> T2(4);
	myMatrices<float> T3(4);
	T1.setArray(T1_array, 4 * 4);
	T2.setArray(T2_array, 4 * 4);
	T3.setArray(T3_array, 4 * 4);
	//����ĩ�˾������ת����
	myMatrices<float> T = T1 * T2 * T3;
	//����ĩ�˾����λ�Ʋ���
	T.setElement(0, 3, _endSpace[0]);
	T.setElement(1, 3, _endSpace[1]);
	T.setElement(2, 3, _endSpace[2]);

	//����Ŀ��ռ����
	workSpace_t = T;

	jointSpace_t = ik();
}

template<uint8_t jNum>
void Arm_Def_s<jNum>::update_Space_t(myMatrices<float> _T)
{
	workArray_t = T2A(_T);
	_T.print();
	workArray_t.print();
	//����Ŀ��ռ����
	workSpace_t = _T;

	jointSpace_t = ik();
}

template<uint8_t jNum>
myMatrices<float> Arm_Def_s<jNum>::fk()
{
	myMatrices<float> T(4);
	for (int i = 0; i < jNum; i++)
	{
		myMatrices<float> T1(4);//alpha��a���
		myMatrices<float> T2(4);//d��theta���
		float _alpha = DH_params.getElement(i, 0);
		float _a = DH_params.getElement(i, 1);
		float _d = DH_params.getElement(i, 2);
		float _theta = DH_params.getElement(i, 3);
		float T1_array[4 * 4] = { 1,		   0,			 0,_a,
								  0,cosf(_alpha),-sinf(_alpha), 0,
								  0,sinf(_alpha), cosf(_alpha), 0,
								  0,		   0,			 0, 1 };
		float T2_array[4 * 4] = { cosf(_theta),-sinf(_theta),0, 0,
								  sinf(_theta), cosf(_theta),0, 0,
											 0,			   0,1,_d,
											 0,			   0,0, 1 };
		T1.setArray(T1_array, 4 * 4);
		T2.setArray(T2_array, 4 * 4);
		myMatrices<float> dT = T1 * T2;
		if (i > 0)
		{
			T = T * dT;//�ҳ�
		}
		else
		{
			T = dT;
		}
	}
	T = T * spaceCali;
	workArray_c = T2A(T);
	return T;
}

//��ͨ�����
template<uint8_t jNum>
myMatrices<float> Arm_Def_s<jNum>::ik()
{
	//�����֮ǰ�ȸ������ռ����Լ��
	for (int i = 0; i < 6; i++)
	{
		if (workArrayCs.getElement(i, 0) != 0 || workArrayCs.getElement(i, 1) != 0)
		{
			float temp = upper::constrain(workArray_t.getElement(0, i), workArrayCs.getElement(i, 0), workArrayCs.getElement(i, 1));
			workArray_t.setElement(0, i, temp);
		}
	}
	myMatrices<float> J(1, jNum);
	if (jNum == 5)
	{
		J.setElement(0, 0, workArray_t.getElement(0, 5));
		J.setElement(0, 4, workArray_t.getElement(0, 3));
		float x4 = sqrtf(powf(workArray_t.getElement(0, 0), 2) + powf(workArray_t.getElement(0, 1), 2)) - cosf(-workArray_t.getElement(0, 4)) * DH_params.getElement(4, 2);
		float z4 = workArray_t.getElement(0, 2) - sinf(-workArray_t.getElement(0, 4)) * DH_params.getElement(4, 2) - DH_params.getElement(0, 2);//���Ҫ��ȥd1�ĸ߶�
		float pow14 = x4 * x4 + z4 * z4;
		float powL2 = powf(DH_params.getElement(2, 1), 2);
		float powL3 = powf(DH_params.getElement(3, 1), 2);
		float alpha = atan2(z4, x4);
		float beta = acosf(upper::constrain((pow14 + powL2 - powL3) / (2.f * DH_params.getElement(2, 1) * sqrtf(pow14)),1));
		J.setElement(0, 1, alpha + beta);
		J.setElement(0, 2, acosf(upper::constrain((powL2 + powL3 - pow14) / (2.f * DH_params.getElement(2, 1) * DH_params.getElement(3, 1)),1)) - 3.1415926f);
		J.setElement(0, 3, 3.1415926f / 2.f - workArray_t.getElement(0, 4) - J.getElement(0, 1) - J.getElement(0, 2));//�����DH��offset���Ʋ�ͬ���˴�����90��
	}
	else if (jNum == 6)
	{
		//����5��6�����λ��
		myMatrices<float> p65(4, 1);
		myMatrices<float> p05(4, 1);
		float a65[4] = { -DH_params.getElement(5,2), 0, 0, 1};
		p65.setArray(a65, 4);
		p05 = workSpace_t * p65;
		float x6 = workArray_t.getElement(0, 0);
		float y6 = workArray_t.getElement(0, 1);
		float z6 = workArray_t.getElement(0, 2);
		float d5 = DH_params.getElement(5, 2);
		float x5 = p05.getElement(0, 0);
		float y5 = p05.getElement(1, 0);
		float z5 = p05.getElement(2, 0);

		//����theta1����ֵ
		float angle_Y1 = atan2(y5, x5);
		J.setElement(0, 0, angle_Y1);
		//����T16��ĩ����̬����
		myMatrices<float> T16(4, 4);
		myMatrices<float> T01(4, 4);
		float T01_array[4 * 4] = { cosf(angle_Y1),-sinf(angle_Y1),0, 0,
								   sinf(angle_Y1), cosf(angle_Y1),0, 0,
												0,			    0,1, 0,
												0,			    0,0, 1 };
		T01.setArray(T01_array, 4 * 4);
		T16 = T01.inverse() * workSpace_t;
		//���ݻ�е�۹������ŷ���ڽ�
		myMatrices<float> A16 = T2A(T16);
		J.setElement(0, 5, A16.getElement(0, 5));
		J.setElement(0, 4, A16.getElement(0, 4) + 3.1415926f / 2.f);//�����DH��offset���Ʋ�ͬ���˴�����90��
		//����ѹ������ά������һ�����
		float x4 = sqrtf(powf(x5, 2) + powf(y5, 2)) - cosf(-A16.getElement(0, 3)) * DH_params.getElement(4, 1);
		float z4 = z5 - sinf(-A16.getElement(0, 3)) * DH_params.getElement(4, 1) - DH_params.getElement(0, 2);//���Ҫ��ȥd1�ĸ߶�
		float pow14 = x4 * x4 + z4 * z4;
		float powL2 = powf(DH_params.getElement(2, 1), 2);
		float powL3 = powf(DH_params.getElement(3, 1), 2);
		float alpha_z = atan2(z4, x4);
		float beta_z = acosf(upper::constrain((pow14 + powL2 - powL3) / (2.f * DH_params.getElement(2, 1) * sqrtf(pow14)),1));
		J.setElement(0, 1, alpha_z + beta_z);
		J.setElement(0, 2, acosf(upper::constrain((powL2 + powL3 - pow14) / (2.f * DH_params.getElement(2, 1) * DH_params.getElement(3, 1)),1)) - 3.1415926f);
		J.setElement(0, 3, -A16.getElement(0, 3) - J.getElement(0, 1) - J.getElement(0, 2));
	}
	else if (jNum == 4)
	{
		//�ȸ��ݸ߶�Լ������pitch_1�ĽǶ�
		float a = atan2(DH_params.getElement(2, 1),DH_params.getElement(2, 2));
		float theta2 = asinf(upper::constrain((DH_params.getElement(0, 2) - workSpace_t.getElement(2, 3)) / sqrtf(powf(DH_params.getElement(2, 1), 2) + powf(DH_params.getElement(2, 2), 2)),1)) - a;
		theta2 += DH_params.getElement(1, 4);
		//������α任�����������
		float pow_s2 = powf(sinf(theta2), 2);
		float pow_c2 = powf(cosf(theta2), 2);
		float c4, s4;
		float s1, c1;
		float theta1,theta3, theta4;
		static float ltheta1 = jointSpace_c.getElement(0,0), ltheta3 = jointSpace_c.getElement(0, 2), ltheta4 = jointSpace_c.getElement(0, 3);
		static float cnt1 = 0,cnt3 = 0,cnt4 = 0;
		if (sinf(theta2) <= zero_check && sinf(theta2) >= -zero_check)
		{
			theta4 = atan2(workSpace_t.getElement(2, 0), workSpace_t.getElement(2, 1));
			theta3 = ltheta3;
			theta1 = ltheta1;
		}
		else
		{
			if (workSpace_t.getElement(2, 0) >= 0)
			{
				theta3 = atan2(-workSpace_t.getElement(2, 2), -sqrtf(powf(workSpace_t.getElement(2, 0), 2) + powf(workSpace_t.getElement(2, 1), 2)));
			}
			else
			{
				theta3 = atan2(-workSpace_t.getElement(2, 2), sqrtf(powf(workSpace_t.getElement(2, 0), 2) + powf(workSpace_t.getElement(2, 1), 2)));
			}
			float pow_s3 = powf(sinf(theta3), 2);
			float pow_c3 = powf(cosf(theta3), 2);
			if ((cosf(theta3) <= zero_check && cosf(theta3) >= -zero_check) || (sinf(theta2) <= zero_check && sinf(theta2) >= -zero_check))
			{
				if ((cosf(theta2) <= zero_check && cosf(theta2) >= -zero_check))
				{
					theta4 = ltheta4;
				}
				else
				{
					c4 = workSpace_t.getElement(2, 1) / cosf(theta2);
					s4 = workSpace_t.getElement(2, 0) / cosf(theta2);
					theta4 = atan2(s4, c4);
				}
			}
			else if ((cosf(theta2) <= zero_check && cosf(theta2) >= -zero_check))
			{
				if ((cosf(theta3) <= zero_check && cosf(theta3) >= -zero_check))
				{
					theta4 = ltheta4;
				}
				else
				{
					c4 = workSpace_t.getElement(2, 0) / (sinf(theta2) * cosf(theta3));
					s4 = -workSpace_t.getElement(2, 1) / (sinf(theta2) * cosf(theta3));
					theta4 = atan2(s4, c4);
				}
			}
			else
			{
				c4 = (sinf(theta2) * cosf(theta3) * workSpace_t.getElement(2, 0) + cosf(theta2) * workSpace_t.getElement(2, 1)) / (pow_s2 * pow_c3 + pow_c2);
				s4 = (cosf(theta2) * c4 - workSpace_t.getElement(2, 1)) / (sinf(theta2) * cosf(theta3));
				theta4 = atan2(s4, c4);
			}
			
			if ((sinf(theta3) <= zero_check && sinf(theta3) >= -zero_check) || (cosf(theta2) <= zero_check && cosf(theta2) >= -zero_check))
			{
				s1 = workSpace_t.getElement(0, 2) / cosf(theta3);
				c1 = -workSpace_t.getElement(1, 2) / cosf(theta3);
				theta1 = atan2(s1, c1);
			}
			else if (cosf(theta3) <= zero_check && cosf(theta3) >= -zero_check)
			{
				s1 = -workSpace_t.getElement(1, 2) / (cosf(theta2) * sinf(theta3));
				c1 = -workSpace_t.getElement(0, 2) / (cosf(theta2) * sinf(theta3));
				theta1 = atan2(s1, c1);
			}
			else
			{
				s1 = (cosf(theta3) * workSpace_t.getElement(0, 2) - cosf(theta2) * sinf(theta3) * workSpace_t.getElement(1, 2)) / (pow_c2 * pow_s3 + pow_c3);
				c1 = (cosf(theta3) * s1 - workSpace_t.getElement(0, 2)) / (cosf(theta2) * sinf(theta3));
				theta1 = atan2(s1, c1);
			}
			//ͳ���ۼ�Ȧ��
			if ((theta1 - ltheta1) > 3.1415926f)
			{
				cnt1--;
			}
			else if ((theta1 - ltheta1) < -3.1415926f)
			{
				cnt1++;
			}
			if ((theta3 - ltheta3) > 3.1415926f)
			{
				cnt3--;
			}
			else if ((theta3 - ltheta3) < -3.1415926f)
			{
				cnt3++;
			}
			if ((theta4 - ltheta4) > 3.1415926f)
			{
				cnt4--;
			}
			else if ((theta4 - ltheta4) < -3.1415926f)
			{
				cnt4++;
			}
		}
		//������һ�εĽ�
		ltheta1 = theta1;
		ltheta3 = theta3;
		ltheta4 = theta4;
		float total_theta1 = theta1 + cnt1 * 2.f * 3.1415926f;
		float total_theta3 = theta3 + cnt3 * 2.f * 3.1415926f;
		float total_theta4 = theta4 + cnt4 * 2.f * 3.1415926f;
		J.setElement(0, 0, total_theta1);
		J.setElement(0, 1, theta2);
		J.setElement(0, 2, total_theta3);
		J.setElement(0, 3, total_theta4);
	}	
		
	for (int i = 0; i < jNum; i++)
	{
		if (jointCs.getElement(i, 0) != 0 || jointCs.getElement(i, 1) != 0)
		{
			float temp = upper::constrain(J.getElement(0, i) - DH_params.getElement(i, 4), jointCs.getElement(i, 0), jointCs.getElement(i, 1));
			J.setElement(0, i, temp);
		}
		else
		{
			J.setElement(0, i, J.getElement(0, i) - DH_params.getElement(i, 4));
		}
	}
	
	return J;
}

//˳��y-z-x
template<uint8_t jNum>
myMatrices<float> Arm_Def_s<jNum>::T2A(myMatrices<float> _T)
{
	myMatrices<float> A(1, 6);
	A.setElement(0, 0, _T.getElement(0, 3));
	A.setElement(0, 1, _T.getElement(1, 3));
	A.setElement(0, 2, _T.getElement(2, 3));
	switch (order_c)
	{
	case rOrder::YZX:
		A.setElement(0, 4, atan2f(_T.getElement(1, 0), sqrtf(powf(_T.getElement(0, 0), 2) + powf(_T.getElement(2, 0), 2))));
		A.setElement(0, 3, atan2f(-_T.getElement(2, 0) / cosf(A.getElement(0, 4)), _T.getElement(0, 0) / cosf(A.getElement(0, 4))));
		A.setElement(0, 5, atan2f(-_T.getElement(1, 2) / cosf(A.getElement(0, 4)), _T.getElement(1, 1) / cosf(A.getElement(0, 4))));
		break;
	case rOrder::ZXY:
		A.setElement(0, 4, atan2f(_T.getElement(2, 1), sqrtf(powf(_T.getElement(2, 0), 2) + powf(_T.getElement(2, 2), 2))));
		A.setElement(0, 3, atan2f(-_T.getElement(2, 0) / cosf(A.getElement(0, 4)), _T.getElement(2, 2) / cosf(A.getElement(0, 4))));
		A.setElement(0, 5, atan2f(-_T.getElement(0, 1) / cosf(A.getElement(0, 4)), _T.getElement(1, 1) / cosf(A.getElement(0, 4))));
		break;
	default: break;
	}
	return A;
}

template<uint8_t jNum>
void Arm_Def_s<jNum>::set_spaceCali(const float _spaceCali[4 * 4])
{
	spaceCali.setArray(_spaceCali, 4 * 4);
}

template<uint8_t jNum>
void Arm_Def_s<jNum>::set_jointCs(const float _acs[2 * jNum])
{
	jointCs.setArray(_acs, 2 * jNum);
}

template<uint8_t jNum>
void Arm_Def_s<jNum>::set_workArrayCs(const float _wcs[2 * 6])
{
	workArrayCs.setArray(_wcs, 2 * 6);
}
