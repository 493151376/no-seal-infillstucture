//说明：这个是为了能尽可能没有拉丝，所以想先打印所有三角形的横，再打印三角形的丿，再捺
//思路是：三角形网格就是一个矩阵，分上半阵和下半阵，再分奇偶行
#include<stdio.h>
void main()
{
	/*int m0 = 2, n0 = 2;

	int m = m0, n = 0;

	for (int i = 1; i <= 5; i++)
	{
		if(i<=3)
		{ 
			if (i % 2 == 0)
			{
				do
				{
					printf("(%d, %d)", n, m);
					m--;
					n--;
				} while (m >= 0);
			}
			else
			{
				do
				{
					printf("(%d, %d)", n, m);
					m++;
					n++;
				} while (m <= 2);
			}
			

			n = 0;
			m = m0 - i;
		
		}
		
		else
		{
			m = 0;
			n = i - 3;
			if (i % 2 == 0)
			{
				do
				{
					printf("(%d, %d)", n, m);
					m--;
					n--;
				} while (n >= 0);
			}
			else
			{
				do
				{
					printf("(%d, %d)", n, m);
					m++;
					n++;
				} while (n <=2);
			}
			


		}
	printf("\n");
			
	
	}*/

	//for (int m = 0; m < 3; m++) //遍历横的
	//{
	//	if (m % 2 == 0)
	//	{
	//		for (int n = 0; n < 3; n++)
	//		{
	//			printf("(%d, %d)", n, m);
	//		}
	//	}
	//	else
	//	{
	//		for (int n = 2; n >= 0; n--)
	//		{
	//			printf("(%d, %d)", n, m);
	//		}
	//	}
	//	printf("\n");
	//}


	//int m0 = 4, n0 = 4;
	//int i0 = 2*m0+1;
	//int m , n;
	//for (int i = 1; i <= i0; i++)//这里是斜着遍历丿
	//{
	//	printf("\n");
	//	if (i <= m0+1)
	//	{
	//		if (i % 2 == 0)
	//		{
	//			n = i-1;
	//			m = m0;
	//			goto even;
	//		}
	//		else
	//		{
	//			n = 0;
	//			m = m0 - i + 1;
	//			goto odd;
	//		}
	//	}
	//	else
	//	{
	//		if (i % 2 == 0)
	//		{
	//			n = n0;
	//			m = i0-i;
	//			goto even;
	//		}
	//		else
	//		{
	//			n = i-n0-1;
	//			m = 0;
	//			goto odd;
	//		}
	//	}
	//even:
	//	do//偶数行是丿，奇数行是提
	//	{
	//		printf("(%d, %d)", n, m);
	//		m--;
	//		n--;
	//	} while (m >= 0 && n >= 0);
	//	continue;
	//odd:	
	//	do
	//	{
	//		printf("(%d, %d)", n, m);
	//		m++;
	//		n++;
	//	} while (m <= m0 && n <= m0);
	//}

	//for (int i = 0; i <= 5; i++)//实验goto语句
	//{
	//	printf("i = %d",i);
	//	if (i % 2 == 0)goto even;
	//	else goto odd;
	//even:
	//	printf("even\n");
	//	continue;
	//odd:
	//	printf("odd\n");
	//}

	
int m0 = 3, n0 = 3;
int i0 = 2 * m0 + 1;
int m, n;

for (int i = 1; i <= i0; i++)//这里是斜着遍历捺
{
	printf("\n");
	if (i <= m0 + 1)
	{
		if (i % 2 == 0)
		{
			n = n0 - i + 1;
			m = m0;
			goto even;
		}
		else
		{
			n = n0;
			m = m0-i+1;
			goto odd;
		}
	}
	else
	{
		if (i % 2 == 0)
		{
			n = 0;
			m = i0 - i;
			goto even;
		}
		else
		{
			n = i0 - i;
			m = 0;
			goto odd;
		}
	}

even:
	do//奇数行要反捺，和前面的提对应，
	{
		printf("(%d, %d)", n, m);
		m--;
		n++;
	} while (m >= 0 && n <= n0);
	continue;
odd:
	do
	{
		printf("(%d, %d)", n, m);
		m++;
		n--;
	} while (m <= m0 && n >= 0);

}

	
}
