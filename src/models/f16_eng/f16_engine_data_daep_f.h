//----------------------------------------------------------------------
//----------------------------------------------------------------------
//
// SMARTIES 
// Simulation Modules for Aircraft Real-Time Embedded Systems
// 
// Copyright (C) 2018-2022  ISAE-SUPAERO
//
// Author: Jean-Baptiste Chaudron
// email:  jean-baptiste.chaudron@isae-supaero.fr
//
//----------------------------------------------------------------------
//----------------------------------------------------------------------


#ifndef __F16_ENGINE_DATA_DAEP_F_HH__
#define __F16_ENGINE_DATA_DAEP_F_HH__

#ifdef __cplusplus
extern "C" 
{
#endif


//-------------------------------------------------------------------
const float thrust_wf_00_wfab_00_mach_alt_f[8][16] = 
{
	{0.0,	1000.0,	 1500.0,	2000.0,	  2500.0,   3000.0,  3500.0,  4000.0,  4500.0,  5000.0,  5500.0,  6000.0,  6500.0, 7000.0, 7500.0, 8000.0 },
	{0.3,	   0.0,	    0.0,	   0.0,	     0.0,      0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,    0.0,    0.0,    0.0 },
	{0.4, 	   0.0,	    0.0,	   0.0,	     0.0,      0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,    0.0,    0.0,    0.0 },
	{0.5,	   0.0,	    0.0,	   0.0,	     0.0,      0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,    0.0,    0.0,    0.0 },
	{0.6,	   0.0,	    0.0,	   0.0,	     0.0,      0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,    0.0,    0.0,    0.0 },
	{0.7,	   0.0,	    0.0,	   0.0,	     0.0,      0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,    0.0,    0.0,    0.0 },
	{0.8,	   0.0,	    0.0,	   0.0,	     0.0,      0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,    0.0,    0.0,    0.0 },
	{0.9,	   0.0,	    0.0,	   0.0,	     0.0,      0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,     0.0,    0.0,    0.0,    0.0 }
};


//-------------------------------------------------------------------
const float thrust_wf_03_wfab_00_mach_alt_f[8][16] = 
{
	{  0.0,  1000.0,  1500.0,  2000.0,  2500.0,  3000.0,  3500.0,  4000.0,  4500.0,  5000.0,  5500.0,  6000.0,  6500.0,  7000.0,  7500.0,  8000.0},
	{  0.3,  12851.5,  13227.2,  13596.4,  13967.5,  14334.1,  14681.2,  15007.9,  15315.4,  15607.5,  15871.1,  16109.7,  16328.8,  16532.1,  16717.1,  16886.5},
	{  0.4,  11761.8,  12157.2,  12547.9,  12942.2,  13324.6,  13686.7,  14027.9,  14349.3,  14652.2,  14937.7,  15194.7,  15428.8,  15644.7,  15845.1,  16028.1},
	{  0.5,  10741.1,  11155.9,  11567.3,  11982.3,  12376.9,  12750.8,  13103.4,  13435.6,  13748.7,  14043.4,  14320.7,  14570.9,  14799,  15009.8,  15205.7},
	{  0.6,  9783.01,  10218.9,  10656.2,  11082.2,  11486.5,  11869.4,  12230.8,  12571.5,  12892.9,  13195.4,  13479.2,  13747.6,  13990.4,  14211.7,  14416},
	{  0.7,  8887.75,  9343.92,  9802.1,  10237.3,  10649.1,  11038.3,  11406.6,  11753.8,  12081.3,  12390.1,  12680.2,  12952.4,  13210.3,  13445.7,  13659.4},
	{  0.8,  8056.8,  8539.31,  8998.63,  9440.81,  9858.87,  10253.6,  10626.4,  10978.7,  11311,  11624.4,  11919.6,  12196.6,  12456.7,  12701.8,  12930.4},
	{  0.9,  7291.11,  7783.15,  8249.62,  8690.17,  9108.93,  9508.76,  9885.84,  10241.9,  10578.1,  10895,  11193.7,  11474.9,  11738.6,  11986.3,  12218.6}
};

//-------------------------------------------------------------------
const float thrust_wf_06_wfab_00_mach_alt_f[8][16] = 
{
	{  0.0,  1000.0,  1500.0,  2000.0,  2500.0,  3000.0,  3500.0,  4000.0,  4500.0,  5000.0,  5500.0,  6000.0,  6500.0,  7000.0,  7500.0,  8000.0},
	{  0.3,  29818.4,  30189.3,  30535.4,  30860.1,  31160.1,  31236.8,  31688.9,  31931.6,  32164.7,  32391.4,  32606.3,  32797.5,  32951.8,  33058.5,  33116.9},
	{  0.4,  28115.4,  28512.9,  28881.4,  29226.6,  29551.4,  29848.1,  30122.7,  30378.9,  30624,  30859.4,  31088.1,  31301.7,  31489.3,  31637.4,  31738.8},
	{  0.5,  26487.8,  26914.6,  27305.8,  27668.9,  28008.7,  28328.8,  28622.3,  28893.8,  29147.4,  29389.8,  29622.9,  29848.2,  30058.8,  30243.2,  30388.3},
	{  0.6,  24928.2,  25378.8,  25798.2,  26181.5,  26536.5,  26868.1,  27181.4,  27469.5,  27735.6,  27983.2,  28219,  28445.7,  28665.7,  28871.9,  29054},
	{  0.7,  23448.5,  23908.2,  24340.8,  24751.4,  25126.6,  25472.1,  25793.8,  26096.5,  26379.6,  26638.2,  26877.8,  27105,  27323.3,  27535.2,  27735.8},
	{  0.8,  22040.3,  22507.8,  22949.8,  23365.9,  23760.6,  24129.2,  24465.6,  24777.1,  25068.5,  25342.8,  25593.8,  25825.6,  26043,  26251.6,  26452.8},
	{  0.9,  20697.6,  21171.2,  21619.7,  22043.1,  22442.3,  22819.6,  23177.9,  23507.1,  23808.7,  24088.5,  24350.6,  24596.1,  24820.4,  25028.8,  25226.5}
};

//-------------------------------------------------------------------
const float thrust_wf_09_wfab_00_mach_alt_f[8][16] = 
{
	{  0.0,  1000.0,  1500.0,  2000.0,  2500.0,  3000.0,  3500.0,  4000.0,  4500.0,  5000.0,  5500.0,  6000.0,  6500.0,  7000.0,  7500.0,  8000.0},
	{  0.3,  45466.2,  45796,  46111.1,  46416.9,  46709,  46979.4,  47211.9,  47391.4,  47511.6,  47569.2,  47565.9,  47514.9,  47420,  47279,  47076.2},
	{  0.4,  43215.8,  43566.1,  43899.1,  44217.7,  44527.1,  44821,  45089,  45318,  45493.7,  45612.2,  45669.4,  45671,  45628.1,  45544.2,  45410.2},
	{  0.5,  41059,  41432.1,  41779.7,  42110.1,  42426.1,  42732.8,  43022.4,  43288.3,  43515.9,  43692.6,  43814.4,  43877.3,  43887.5,  43856.4,  43784.7},
	{  0.6,  38986.2,  39383,  39750.8,  40092.3,  40415.7,  40724.5,  41021.8,  41305.3,  41567.9,  41797.1,  41978.7,  42108.2,  42181.4,  42204.3,  42185.9},
	{  0.7,  36992.8,  37448.1,  37797.8,  38158.6,  38492.5,  38805.7,  39102.8,  39388,  39662.8,  39921.3,  40152.6,  40341.6,  40481.8,  40569.5,  40607},
	{  0.8,  35085.5,  35511.1,  35910.5,  36291.2,  36643.9,  36969.5,  37271.1,  37555.5,  37826.9,  38090.8,  38342.1,  38573.2,  38770.7,  38923.6,  39029.6},
	{  0.9,  33232.9,  33685.1,  34100.4,  34487.8,  34861.1,  35195.4,  35513.3,  35805.2,  36076.9,  36335.7,  36584.9,  36826.3,  37053.9,  37257.7,  37425.1}
};

//-------------------------------------------------------------------
const float thrust_wf_12_wfab_00_mach_alt_f[8][16] = 
{
	{  0.0,  1000.0,  1500.0,  2000.0,  2500.0,  3000.0,  3500.0,  4000.0,  4500.0,  5000.0,  5500.0,  6000.0,  6500.0,  7000.0,  7500.0,  8000.0},
	{  0.3,  60429.2,  60753,  61033.4,  61252.3,  61404.3,  61481.4,  61486.2,  61431.7,  61322.3,  61161.2,  60930,  60639.2,  60208.3,  59318.5,  58285.6},
	{  0.4,  57621.6,  57975.9,  58301.2,  58580.4,  58799.5,  58952.6,  59031.4,  59042.5,  58998.7,  58903.5,  58754.7,  58536.9,  58265.8,  57770.8,  56856.1},
	{  0.5,  54916.3,  55288.1,  55641,  55966.1,  56247.7,  56471.1,  56630.5,  56717.7,  56740.9,  56711.4,  56633.2,  56503,  56305.8,  56055.7,  55556.8},
	{  0.6,  52326.3,  52700.9,  53065.5,  53412.7,  53736.8,  54022.5,  54253.7,  54424,  54525.8,  54565.1,  54553.3,  54494.2,  54387.2,  54215.7,  53993.4},
	{  0.7,  49847.7,  50228,  50592.1,  50944,  51282.9,  51603,  51892.3,  52133.4,  52318.7,  52439.8,  52498.7,  52506.8,  52468.9,  52387.5,  52246.8},
	{  0.8,  47466.5,  47861.1,  48231.5,  48582.9,  48919.6,  49246.6,  49557.8,  49846.6,  50097.3,  50297.8,  50442.8,  50525.6,  50555.6,  50541.1,  50484.7},
	{  0.9,  45161.8,  45576.9,  45965.4,  46326.4,  46664.7,  46986.6,  47295.4,  47593.6,  47875.6,  48132.8,  48349.3,  48517.2,  48628.5,  48684.3,  48695}
};

//-------------------------------------------------------------------
const float thrust_wf_15_wfab_00_mach_alt_f[8][16] = 
{
	{  0.0,  1000.0,  1500.0,  2000.0,  2500.0,  3000.0,  3500.0,  4000.0,  4500.0,  5000.0,  5500.0,  6000.0,  6500.0,  7000.0,  7500.0,  8000.0},
	{  0.3,  74802.1,  74956.4,  75022.1,  75012.9,  74941,  74808,  74616,  74348.4,  74008.8,  73546.1,  72586.9,  71363.5,  70110.6,  68820.6,  67473.7},
	{  0.4,  71568,  71802.9,  71961.3,  72035.7,  72040.4,  71983.9,  71869.8,  71696.6,  71446,  71134.1,  70630.5,  69617,  68471.6,  67287.9,  66062.8},
	{  0.5,  68404.2,  68713.3,  68957.2,  69127.9,  69218.2,  69240.7,  69203.7,  69111.8,  68962.5,  68737.5,  68455.3,  67965.7,  66990.6,  65916.9,  64802.2},
	{  0.6,  65307,  65673,  65990.9,  66248.1,  66437.8,  66548.8,  66592.7,  66579,  66512.2,  66391.1,  66199.4,  65947.9,  65535,  64653.3,  63644.4},
	{  0.7,  62306.3,  62693.7,  63058.2,  63385.2,  63658.1,  63869.3,  64007,  64076,  64088.6,  64048.2,  63957.9,  63804.2,  63584.9,  63272.5,  62549.9},
	{  0.8,  59429.9,  59817.5,  60192.9,  60550.4,  60881.9,  61169.9,  61403.1,  61573,  61671.1,  61712.2,  61700.2,  61639.5,  61525.5,  61344.2,  61110.5},
	{  0.9,  56680.3,  57070,  57440.3,  57798.2,  58143.6,  58471.7,  58770.7,  59023.8,  59223.2,  59358.1,  59430.4,  59449.3,  59418.2,  59342,  59207.7}
};

//-------------------------------------------------------------------
const float thrust_wf_18_wfab_00_mach_alt_f[8][16] = 
{
	{  0.0,  1000.0,  1500.0,  2000.0,  2500.0,  3000.0,  3500.0,  4000.0,  4500.0,  5000.0,  5500.0,  6000.0,  6500.0,  7000.0,  7500.0,  8000.0},
	{  0.3,  88273,  88216.7,  88088.6,  87898.9,  87633.3,  87277.2,  86858.1,  86135.1,  84828.1,  83429.8,  81981.2,  80486.7,  78953.4,  77532.5,  76126.9},
	{  0.4,  84742.5,  84775.1,  84737.1,  84630.7,  84466,  84221.8,  83893,  83490.7,  82693.8,  81412.4,  80102.1,  78732.9,  77314.9,  75902.9,  74573.5},
	{  0.5,  81292.6,  81424.6,  81478,  81462.6,  81383.1,  81244.2,  81030.7,  80736,  80364.7,  79604.7,  78398.9,  77165.6,  75876.8,  74536,  73218.8},
	{  0.6,  77874.7,  78119.7,  78278.2,  78356.8,  78368.3,  78316.7,  78208.5,  78032.1,  77775.5,  77454.3,  76821.4,  75712.6,  74556.2,  73346.6,  72085.5},
	{  0.7,  74484.9,  74817.3,  75086.7,  75277.8,  75385.8,  75428,  75406.9,  75331.3,  75196.2,  74985,  74710.9,  74254,  73300.8,  72211.5,  71081.2},
	{  0.8,  71140.3,  71529.3,  71877.4,  72168.9,  72395,  72541,  72616.9,  72631,  72588.2,  72494.3,  72332.3,  72098.7,  71788.3,  71095.2,  70072.9},
	{  0.9,  67899.2,  68299.1,  68683.2,  69039.6,  69351.9,  69608.4,  69800.5,  69914,  69965.1,  69959.1,  69902,  69788.6,  69608.6,  69371.8,  68966.7}
};

//-------------------------------------------------------------------
const float thrust_wf_18_wfab_03_mach_alt_f[8][16] = 
{
	{  0.0,  1000.0,  1500.0,  2000.0,  2500.0,  3000.0,  3500.0,  4000.0,  4500.0,  5000.0,  5500.0,  6000.0,  6500.0,  7000.0,  7500.0,  8000.0},
	{  0.3,  91962.6,  91794.8,  91584,  91318,  91007.8,  90627.2,  90149.1,  89598.5,  88768.8,  87377.7,  85896.3,  84457.2,  83075,  81834.1,  80686.6},
	{  0.4,  88669.9,  88574.9,  88436.4,  88249.3,  88013.3,  87730.6,  87370.8,  86917.8,  86387.7,  85495.6,  84106.6,  82741.7,  81385.2,  80122.1,  78984.1},
	{  0.5,  85484.7,  85476.1,  85407.3,  85292.4,  85133.1,  84925.9,  84671.7,  84342.9,  83924.9,  83430.5,  82567.7,  81245.4,  79961.5,  78681.2,  77504.8},
	{  0.6,  82356.2,  82451.6,  82469.6,  82428.2,  82336.3,  82205.9,  82026.8,  81802.8,  81512.3,  81137.3,  80696.1,  79946.5,  78720.5,  77497.9,  76283.4},
	{  0.7,  79227.2,  79441.5,  79572.6,  79622.6,  79608.9,  79542.7,  79439.5,  79288.8,  79094.8,  78847.1,  78524.6,  78134.1,  77545.9,  76474.4,  75291.4},
	{  0.8,  76097,  76402.2,  76646,  76816.2,  76906.5,  76924.8,  76887.1,  76806.5,  76686.7,  76522,  76316.9,  76048.9,  75703.3,  75264.6,  74438.8},
	{  0.9,  73002.2,  73361.2,  73679.4,  73948.2,  74155.1,  74290.5,  74348.5,  74345.2,  74290.9,  74198.7,  74065.2,  73895.7,  73680.1,  73393.8,  73039.2}
};

//-------------------------------------------------------------------
const float thrust_wf_18_wfab_06_mach_alt_f[8][16] = 
{
	{  0.0,  1000.0,  1500.0,  2000.0,  2500.0,  3000.0,  3500.0,  4000.0,  4500.0,  5000.0,  5500.0,  6000.0,  6500.0,  7000.0,  7500.0,  8000.0},
	{  0.3,  95072.6,  94857.1,  94572.5,  94253.8,  93912.7,  93500.5,  92972.1,  92306.6,  91502.4,  90461.9,  89115.6,  87758.4,  86504.5,  85259.4,  83981.4},
	{  0.4,  91940.7,  91811.2,  91617.4,  91359.4,  91073.2,  90761.1,  90368,  89858.4,  89207.9,  88418.6,  87362.2,  86035.5,  84808.3,  83632.4,  82442.2},
	{  0.5,  88914.4,  88875.5,  88769.3,  88601.6,  88370.3,  88110.3,  87827.7,  87465.7,  86990.1,  86374.7,  85622.8,  84605.1,  83346.6,  82189.6,  81074.2},
	{  0.6,  85962.5,  86010.7,  85996.7,  85916.8,  85778.4,  85574,  85336.7,  85082.9,  84763.5,  84334.6,  83770,  83077.9,  82143.6,  80953.1,  79847.4},
	{  0.7,  83073,  83191.9,  83261.3,  83274.1,  83224.1,  83116.4,  82944.1,  82726.7,  82497,  82222.9,  81851.9,  81352.6,  80731,  79916.1,  78834.3},
	{  0.8,  80221.2,  80412.7,  80550.8,  80640.9,  80680.2,  80663.4,  80586.4,  80451.4,  80260.6,  80046.3,  79811.5,  79501.9,  79079.3,  78531.7,  77847.9},
	{  0.9,  77349.8,  77635.9,  77858.5,  78019.1,  78127.5,  78191.2,  78206.9,  78164.9,  78066.3,  77914,  77716,  77504.8,  77251.7,  76909.6,  76448.5}
};

//-------------------------------------------------------------------
const float thrust_wf_18_wfab_09_mach_alt_f[8][16] = 
{
	{  0.0,  1000.0,  1500.0,  2000.0,  2500.0,  3000.0,  3500.0,  4000.0,  4500.0,  5000.0,  5500.0,  6000.0,  6500.0,  7000.0,  7500.0,  8000.0},
	{  0.3,  97780.5,  97554.1,  97239.2,  96844.6,  96413.7,  95961.5,  95376.4,  94647.1,  93783.1,  92803.5,  91538.6,  90106.2,  88683.8,  87330.9,  86101.8},
	{  0.4,  94770.5,  94644.8,  94434.4,  94139.3,  93769.3,  93378.5,  92946.8,  92368.1,  91664.1,  90845.3,  89834.4,  88587.8,  87215.2,  85874,  84617.6},
	{  0.5,  91867.3,  91826.8,  91721.8,  91535.5,  91267.5,  90926.5,  90565.4,  90162.5,  89614.2,  88948.1,  88172.6,  87190.2,  85990.1,  84677.6,  83404.2},
	{  0.6,  89082.1,  89086.3,  89062.7,  88982.6,  88826.9,  88591.4,  88283.9,  87944.3,  87577.7,  87080.7,  86461,  85726.6,  84827,  83696.3,  82438.3},
	{  0.7,  86407.1,  86450.7,  86462.4,  86450,  86396.4,  86276.4,  86079.5,  85810.8,  85488.8,  85156.3,  84727,  84162.2,  83507.1,  82691.7,  81648},
	{  0.8,  83786.4,  83900.2,  83956.5,  83975.6,  83969.2,  83936.5,  83851.9,  83698.4,  83472.7,  83180.1,  82864,  82504.4,  82011.8,  81414.2,  80727.7},
	{  0.9,  81146.7,  81355,  81498,  81576.8,  81607,  81605.8,  81581.7,  81527.9,  81418.2,  81240.5,  80992.5,  80687.2,  80372.4,  79973.1,  79443.5}
};

//-------------------------------------------------------------------
const float thrust_wf_18_wfab_12_mach_alt_f[8][16] = 
{
	{  0.0,  1000.0,  1500.0,  2000.0,  2500.0,  3000.0,  3500.0,  4000.0,  4500.0,  5000.0,  5500.0,  6000.0,  6500.0,  7000.0,  7500.0,  8000.0},
	{  0.3,  100089,  99860.6,  99552,  99128.6,  98648,  98129,  97490.3,  96695.5,  95836.5,  94819.9,  93594.2,  91986.7,  90358.1,  88921.9,  87710},
	{  0.4,  97235.6,  97072.9,  96863.5,  96565.1,  96161.8,  95713,  95215.6,  94534.1,  93831.2,  92980.7,  91977.8,  90696.8,  89148.5,  87593.9,  86326.6},
	{  0.5,  94507.9,  94411.9,  94268,  94078.1,  93804.5,  93428.8,  93008.6,  92539.4,  91892.8,  91225.4,  90408.8,  89439.6,  88180.7,  86696.5,  85208.8},
	{  0.6,  91893.4,  91854.6,  91771.9,  91645.4,  91475.3,  91236.9,  90896.6,  90501.7,  90067.4,  89469.8,  88842.5,  88081.5,  87165.7,  85991.4,  84569.2},
	{  0.7,  89384.5,  89384.8,  89357.5,  89287.5,  89178.7,  89027.7,  88827.2,  88534.7,  88164.3,  87762.3,  87277.5,  86643.5,  85955.1,  85108.8,  84070.6},
	{  0.8,  86956.6,  87008.5,  87015.4,  86992.2,  86936.7,  86845.2,  86711.2,  86541,  86302.8,  85969,  85591.9,  85170.4,  84592.4,  83984.5,  83217.2},
	{  0.9,  84517.1,  84661.6,  84742.5,  84764.9,  84743.5,  84698.6,  84619.5,  84506.3,  84356,  84164.2,  83891.1,  83538.6,  83157.3,  82702.7,  82106.7}
};

//-------------------------------------------------------------------
const float thrust_wf_18_wfab_15_mach_alt_f[8][16] = 
{
	{  0.0,  1000.0,  1500.0,  2000.0,  2500.0,  3000.0,  3500.0,  4000.0,  4500.0,  5000.0,  5500.0,  6000.0,  6500.0,  7000.0,  7500.0,  8000.0},
	{  0.3,  102046,  101766,  101410,  100982,  100489,  99923.5,  99467.7,  98634.5,  97614.5,  96388.6,  95060.4,  93757.3,  92255.9,  90629.7,  89070.7},
	{  0.4,  99344.8,  99143.6,  98881.1,  98541.2,  98127.3,  97664.7,  97147,  96658.3,  95834.6,  94814.1,  93604.4,  92335.1,  91101.4,  89579.7,  88007.9},
	{  0.5,  96809.9,  96640.1,  96456.9,  96214.9,  95895.4,  95503.6,  95066.5,  94580.3,  94111,  93318.6,  92329.9,  91158.5,  89941.1,  88751.9,  87272.7},
	{  0.6,  94404.2,  94284.8,  94124.1,  93954.9,  93737.1,  93442.6,  93079.9,  92662.6,  92193.3,  91789,  91048.3,  90119.2,  89001.2,  87826.2,  86693.7},
	{  0.7,  92063.2,  92021.9,  91923.3,  91772.1,  91608.9,  91416.9,  91152.7,  90822.4,  90426.2,  89965.4,  89606.6,  88975,  88129.2,  87084.2,  85947.6},
	{  0.8,  89755.5,  89790.4,  89770.2,  89694,  89564.3,  89400.8,  89223.8,  88993.3,  88693.5,  88333.5,  87940.6,  87508.7,  87049.5,  86301.5,  85356.5},
	{  0.9,  87511.4,  87585,  87621.8,  87623.7,  87572.5,  87467.4,  87315.7,  87137.5,  86937.3,  86676.4,  86356.5,  85981.9,  85539.4,  85185.9,  84578.2}
};

//-------------------------------------------------------------------
const float thrust_wf_18_wfab_18_mach_alt_f[8][16] = 
{
	{  0.0,  1000.0,  1500.0,  2000.0,  2500.0,  3000.0,  3500.0,  4000.0,  4500.0,  5000.0,  5500.0,  6000.0,  6500.0,  7000.0,  7500.0,  8000.0},
	{  0.3,  103762,  103409,  102990,  102474,  102038,  101507,  100953,  100344,  99142.5,  97824.1,  96518.6,  95237.4,  93960.9,  92518.5,  90836.4},
	{  0.4,  101240,  100961,  100625,  100223,  99741.8,  99330.9,  98820.5,  98292.3,  97607.6,  96403.8,  95139.7,  93900.2,  92676.9,  91451.3,  89955.3},
	{  0.5,  98853.9,  98647,  98386.4,  98068.6,  97688,  97231.1,  96840.6,  96354.5,  95848,  95171.9,  94008.3,  92790,  91603.3,  90425.3,  89243.5},
	{  0.6,  96593.1,  96437.9,  96246.1,  96005.8,  95709.3,  95351.5,  94909.8,  94539.1,  94078.2,  93593.6,  93010.8,  91902.4,  90724.5,  89579.7,  88437.7},
	{  0.7,  94399.2,  94324.7,  94186.9,  94009.2,  93790.1,  93515.7,  93182.1,  92757.2,  92380.8,  91965.1,  91499.4,  90987.8,  90036.8,  88900,  87783.4},
	{  0.8,  92266.8,  92238.4,  92184.3,  92074.3,  91908.6,  91706.3,  91454.7,  91150.5,  90792,  90364.7,  89996.2,  89540,  89058.7,  88359.4,  87275.8},
	{  0.9,  90192.1,  90218.5,  90198.8,  90145.7,  90064.2,  89922.1,  89733.4,  89505.3,  89228.8,  88899.1,  88522.3,  88103.3,  87698,  87236.8,  86738.1}
};

//-------------------------------------------------------------------
const float thrust_wf_18_wfab_21_mach_alt_f[8][16] = 
{
	{  0.0,  1000.0,  1500.0,  2000.0,  2500.0,  3000.0,  3500.0,  4000.0,  4500.0,  5000.0,  5500.0,  6000.0,  6500.0,  7000.0,  7500.0,  8000.0},
	{  0.3,  105291,  104890,  104420,  103928,  103440,  102898,  102156,  101291,  100433,  99279,  97994.8,  96697.5,  95388.8,  94067.8,  92625.6},
	{  0.4,  102912,  102587,  102207,  101754,  101296,  100830,  100292,  99541.7,  98707.7,  97906.2,  96690.7,  95459.5,  94203.3,  92939.4,  91664},
	{  0.5,  100675,  100423,  100115,  99754.3,  99324.6,  98886.4,  98438.6,  97919.6,  97189.5,  96388.6,  95622.4,  94427.8,  93238.3,  92019.2,  90789.8},
	{  0.6,  98534.8,  98365.7,  98132.3,  97842.7,  97498.7,  97098.4,  96667.9,  96235.5,  95752.9,  95066.4,  94293.5,  93543.1,  92442.6,  91287.4,  90099.5},
	{  0.7,  96476.6,  96367.8,  96216.8,  96005.1,  95733.9,  95411.8,  95044.3,  94616.1,  94193.1,  93738.1,  93133,  92391,  91643.9,  90688.1,  89565},
	{  0.8,  94504.4,  94438.5,  94334.6,  94197,  94013,  93765.1,  93468.3,  93123,  92715.7,  92288,  91854.2,  91344.6,  90651.1,  89909,  89131},
	{  0.9,  92606.8,  92581.2,  92511.8,  92418.2,  92286.9,  92121.1,  91903.1,  91629.6,  91309.3,  90942,  90506.9,  90077.8,  89620.6,  89036.4,  88318.6}
};

//-------------------------------------------------------------------
const float thrust_wf_18_wfab_24_mach_alt_f[8][16] = 
{
	{  0.0,  1000.0,  1500.0,  2000.0,  2500.0,  3000.0,  3500.0,  4000.0,  4500.0,  5000.0,  5500.0,  6000.0,  6500.0,  7000.0,  7500.0,  8000.0},
	{  0.3,  106671,  106235,  105828,  105344,  104784,  104104,  103261,  102275,  101352,  100563,  99489.3,  98167.3,  96831.6,  95533.5,  94267.4},
	{  0.4,  104426,  104063,  103656,  103272,  102793,  102244,  101560,  100727,  99768.9,  98917.9,  98190.9,  97015,  95725.5,  94454.3,  93221.4},
	{  0.5,  102307,  102035,  101688,  101299,  100930,  100470,  99937.2,  99276.9,  98466.2,  97542.7,  96731.2,  96032.8,  94853.4,  93608.2,  92382.6},
	{  0.6,  100288,  100093,  99844.2,  99514.4,  99132.6,  98773.4,  98343.6,  97835.6,  97217.6,  96443.1,  95553.1,  94747.3,  94059.2,  92963.5,  91758.9},
	{  0.7,  98366,  98227.3,  98046,  97814.4,  97511.3,  97129.6,  96778.3,  96383.4,  95901.8,  95336.8,  94617.7,  93769.4,  92944.1,  92245.6,  91303.5},
	{  0.8,  96530,  96430.4,  96305.2,  96129.9,  95915.3,  95644.8,  95305.8,  94920.5,  94551.5,  94102.4,  93588.9,  92946.7,  92167.6,  91314.9,  90580.9},
	{  0.9,  94780.2,  94718.5,  94616.8,  94489.5,  94327.7,  94124.9,  93876,  93573.5,  93141.6,  92815,  92415.5,  91937.5,  91376,  90674.2,  89853.5}
};

//-------------------------------------------------------------------
const float thrust_wf_18_wfab_27_mach_alt_f[8][16] = 
{
	{  0.0,  1000.0,  1500.0,  2000.0,  2500.0,  3000.0,  3500.0,  4000.0,  4500.0,  5000.0,  5500.0,  6000.0,  6500.0,  7000.0,  7500.0,  8000.0},
	{  0.3,  107931,  107503,  107067,  106633,  106040,  105252,  104300,  103325,  102421,  101589,  100829,  99781.4,  98477.1,  97149.4,  95866.4},
	{  0.4,  105777,  105399,  105008,  104594,  104177,  103565,  102770,  101839,  100921,  100072,  99284.7,  98583.8,  97425.5,  96163.5,  94884.5},
	{  0.5,  103776,  103480,  103114,  102738,  102335,  101930,  101338,  100571,  99664.8,  98787.1,  97969.5,  97210,  96536.3,  95375.9,  94146.5},
	{  0.6,  101891,  101657,  101381,  101045,  100658,  100262,  99865.9,  99326.3,  98612.1,  97736.6,  96877,  96074.8,  95326,  94659.2,  93586.2},
	{  0.7,  100100,  99927.9,  99706.3,  99447.7,  99123.1,  98743.1,  98351.2,  97954.7,  97488.2,  96845.9,  96018.4,  95160.6,  94364.1,  93610.3,  92933.6},
	{  0.8,  98387,  98265.5,  98102.4,  97896.5,  97641.8,  97343.3,  96970.7,  96583,  96176.9,  95776.5,  95212.7,  94468.2,  93620.4,  92820.2,  92048},
	{  0.9,  96782,  96684,  96545.1,  96391.6,  96191,  95954.1,  95668.9,  95330.4,  94931.5,  94526.2,  94117.3,  93665.7,  93029.3,  92228.8,  91408.9}
};

//-------------------------------------------------------------------
const float thrust_wf_18_wfab_30_mach_alt_f[8][16] = 
{
	{  0.0,  1000.0,  1500.0,  2000.0,  2500.0,  3000.0,  3500.0,  4000.0,  4500.0,  5000.0,  5500.0,  6000.0,  6500.0,  7000.0,  7500.0,  8000.0},
	{  0.3,  109074,  108659,  108200,  107742,  107231,  106452,  105481,  104523,  103651,  102770,  101942,  101215,  100165,  98856.2,  97533.6},
	{  0.4,  107005,  106654,  106244,  105805,  105365,  104857,  104049,  103111,  102213,  101376,  100526,  99761.4,  99089.4,  97909.3,  96640.3},
	{  0.5,  105115,  104791,  104453,  104053,  103628,  103209,  102718,  101936,  101030,  100160,  99342.4,  98520.6,  97786.4,  97146.3,  95946.4},
	{  0.6,  103349,  103085,  102775,  102437,  102055,  101637,  101235,  100783,  100067,  99192.8,  98323.7,  97513.9,  96717.3,  95992.4,  95358.8},
	{  0.7,  101692,  101483,  101219,  100929,  100592,  100223,  99811,  99411.9,  98991.4,  98389.7,  97563.6,  96685,  95867.6,  95087.4,  94354.7},
	{  0.8,  100110,  99952.2,  99751.2,  99501.1,  99215.3,  98886,  98523.1,  98126.2,  97718.6,  97318.6,  96841.5,  96101.7,  95229.5,  94383.6,  93614.4},
	{  0.9,  98632.1,  98500.9,  98324,  98132.5,  97897.3,  97614.1,  97301.7,  96935.8,  96556.1,  96146,  95741.5,  95321.6,  94749.1,  93933.9,  93048.1}
};



#ifdef  __cplusplus
}
#endif // __cplusplus

#endif // __F16_ENGINE_DATA_F_HH__