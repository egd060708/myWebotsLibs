#pragma once

#include "myMatrices.h"
USING_NAMESPACE_MM

//�������˲���������
template<class T,uint8_t xNum,uint8_t uNum>
class kalmanFilter {
private:
	myMatrices<T> A = myMatrices<T>(xNum);//״̬ת�ƾ���
	myMatrices<T> B = myMatrices<T>(xNum, uNum);//�������
	myMatrices<T> Q = myMatrices<T>(xNum);//��������Э�������
	myMatrices<T> R = myMatrices<T>(xNum);//��������Э�������
	myMatrices<T> H = myMatrices<T>(xNum);//��������
	myMatrices<T> P = myMatrices<T>(xNum);//�������Э�������
	myMatrices<T> x = myMatrices<T>(xNum, 1);//����ÿ�ε�״̬
public:
	//���캯��
	kalmanFilter() { P.eye(); }
	//����״̬�ռ䷽�̣�����������
	void setFunc(const T A_array[xNum * xNum], const T B_array[xNum * uNum],const T H_array[xNum * xNum])
	{
		A.setArray(A_array, xNum * xNum);
		B.setArray(B_array, xNum * uNum);
		H.setArray(H_array, xNum * xNum);
	}
	//����Э�������(ע�⣬Э���������Ժ�С��������Ϊ��)
	void setConv(const T Q_array[xNum * xNum], const T R_array[xNum * xNum])
	{
		Q.setArray(Q_array, xNum * xNum);
		R.setArray(R_array, xNum * xNum);
	}
	//��⿨�����˲�
	void f(const T u_array[uNum], const T z_array[xNum])
	{
		myMatrices<T> u(uNum, 1);
		myMatrices<T> z(xNum, 1);
		u.setArray(u_array, uNum);
		z.setArray(z_array, xNum);
		//��������״̬����
		myMatrices<T> x_minus = A * x + B * u;
		//�����������Э����
		myMatrices<T> P_minus = A * P * A.transpose() + Q;
		//���㿨��������
		myMatrices<T> temp = H * P_minus * H.transpose() + R;
		myMatrices<T> K = P_minus * H.transpose() * temp.inverse();
		//���º������
		x = x_minus + K * (z - H * x_minus);
		//���º������Э����
		myMatrices<T> E(xNum);
		E.eye();
		P = (E - K * H) * P_minus;
	}
	const myMatrices<T> getOut()
	{
		return x;
	}
};
