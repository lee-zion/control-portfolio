double deg_table[11] = { -15.0, -12.0,   -9.0,   -6.0, -3.0,  0.0,  3.0, 6.0,  9.0,  12.0, 15.0};
double V_table[11]   = {1.1789,  1.0940, 0.7883 , 0.6, 0.3, 0.1096, 0.3008 , 0.6628, 0.8322,  1.0786 ,  1.2131};

double Ultra_Table (double DIR, double DOA)
{
	int i = 0;
	double Deg_Hat = 0.0;
	
	if (DIR > 2.5) // 0 ~ 15[deg]
	{
		for (i = 5; i < 10; i++)
		{
			 if ((V_table[i] <= DOA) && (V_table[i+1] >= DOA))
				Deg_Hat = deg_table[i] + ( DOA - V_table[i] ) / (V_table[i+1] - V_table[i] ) * 3;
		}
	}
	else if (DIR < 2.5) // -15 ~ 0[deg]
	{
		for(i = 5; i > 0; i--)
		{
			if ((V_table[i] <= DOA) && (V_table[i-1] >= DOA))
				Deg_Hat = deg_table[i] - ( DOA - V_table[i] ) / (V_table[i-1] - V_table[i] ) * 3;
		}
	}
	return Deg_Hat ;

}

