#!/bin/sh
#----How To Use----#
# $./throughput3.sh <tracefile_name> <outputfile_name> -1 <packet_type>

#$3 time $33 dest-node $35 packet_type $37 packet_size



#第1コマンドライン引数として、スループットを解析したいトレースファイル名を指定する
tracefile_name=$1
#第2コマンドライン引数として、出力ファイル名を指定する
output_name=$2
#第3コマンドライン引数として、スループットを求めたい受信ノード番号を指定する。
#-1を指定した場合は、ネットワーク全体のスループットを求める
node_number=$3
#第4コマンドライン引数として、スループットを求めたいパケットの種別を指定する。
#-1を指定した場合は、すべてのパケットの種別のスループットを求める
packet_type=$4
#第4,5コマンドライン引数として、スループットを求めたい時間の範囲を指定する。(X ~ Y)
#−1を指定した場合は、最初から　または　最後までとなる。
#start_time=$5
#end_time=$6

#シミュレーション時間を格納する変数
time=0
#ファイルから読み込んだその1行の現在時刻を格納する
now_time=0
#送信されたパケットサイズの格納する変数
packet_size=0
#ファイルから読み込んだ、その一行でのパケットサイズを格納する
now_size=0

#トレースファイルから　時間、受信ノード、パケット種別、パケットサイズを抜き出す
#tmp.thp , tmp2.thp は一時的な出力ファイル
awk '{ print $2 " " $3 " " $7 " " $8 " " $4 " " $1;}' $tracefile_name > tmp.thp

#トレースファイルから不要な行を破棄する
#無線用の新トレースファイルの場合、必要なのはRTR層のデータのみ
#パケットを受信したことを示す[r]も同時に検索
grep -i "RTR r" tmp.thp > tmp2.thp
mv tmp2.thp tmp.thp



#指定したノード番号がある場合、それを含まない行を破棄する
#if test $3 -ne -1
#then
#	grep $3* $output_name
#fi
#指定したパケットの種別がある場合、それを含まない行を破棄する
if test $4 != -1
then 
	grep -i $4 tmp.thp > tmp2.thp
	mv tmp2.thp tmp.thp
fi
#指定した時間の範囲がある場合、それを含まない行を破棄する

#条件に応じた抽出を行った後、スループットを求めグラフ化する
mv tmp.thp $output_name.log

#スループットファイルの初期化
echo "" > $output_name.thp
while read line
do
	#抽出が完了したファイルから一行ずつ時間とパケット受信時間を計算し、一秒ごとのスループットを書き出す
	now_time=`echo $line | cut -d ' ' -f1`
	now_time=`echo $now_time | sed s/\.[0-9,]*$//g`
	if [ $now_time -lt $time ]
	then
		now_size=`echo $line | cut -d' ' -f4`
		packet_size=`expr $packet_size + $now_size` 
	else
		echo "`expr $time + 1 ` $packet_size" >> $output_name.thp
		echo "`expr $time + 1 ` $packet_size"
		packet_size=0
		time=`expr $time + 1`
		packet_size=`expr $packet_size + $now_size`
	fi
done < $output_name.log

#作成したスループットファイルを元に、gnuplotを使ってグラフ化する
#pause -1 を指定しているのは、出力した画面が一瞬で消えてしまわないようにするため
gnuplot -e "
	set xtics 0 , 1 ;
	set title '$output_name.thp   throughput of $packet_type' ;
	set xlabel 'second' ;
	set ylabel 'throughput : bite' ;
	plot '$output_name.thp' with line ;
	save '$output_name.plt' ;
	"
