# coding: UTF-8
#
# y3module.py
#
# Wi-SUNモジュールBP35A1(ROHM) 通信クラス Y3Module
#
# Copyright(C) 2016 pi@blue-black.ink
#

import datetime
import serial
import threading
import time
import sys
import user_conf

class Y3Module(threading.Thread):
    """Wi-SUN Module BP35A1(ROHM) 通信クラス"""
    def __init__(self):
        """コンストラクタ"""
        super().__init__()
        self.Y3_UDP_ECHONET_PORT = 3610 # ECHONET UDPポート
        self.Y3_UDP_PANA_PORT = 716     # PANAポート
        self.Y3_TCP_ECHONET_PORT = 3610 # TCPポート
        
        self.msg_list_queue = []        # 受信データ用リスト
        self.term_flag = False          # run()の終了フラグ

        self.uart_hdl = None            # UART
        self.uart_dev = None
        self.uart_baud = 115200

        self.search = {                 # write()用, UART送信後の受信待ちデータ
            'search_words': [],             # UART送信後の受信待ちデータリスト
            'ignore_intermidiate': False,   # 途中の受信データを無視する
            'found_word_list': [],          # 受け取った受信待ちデータリスト
            'start_time': None,             # UART送信時のtime
            'timeout': 0}                   # 設定タイムアウト時間[s]

        self.msg_list_lock = threading.Lock()   # msg_listの排他制御用


    def set_reset(self):
        """SKRESET"""
        res = self.write(b'SKRESET\r\n', [['OK', 'FAIL']], ignore = True, timeout = 3)
        return


    def clear_sk(self):
        """コマンド入力バッファクリア"""
        res = self.write(b'\r\n')
        return


    def get_ver(self):
        """SKVER"""
        res = self.write(b'SKVER\r\n', ['EVER', 'OK'], ignore = True, timeout = 3)
        return res[0]['VERSION']


    def req_echo(self, ip6):
        """SKPING"""
        if user_conf.UDG1WSNE:
            res = self.write(b'SKPING 0 ' + ip6.encode() + b'\r\n', ['OK', 'EPONG'], ignore = True, timeout = 3)
        else:
            res = self.write(b'SKPING ' + ip6.encode() + b'\r\n', ['OK', 'EPONG'], ignore = True, timeout = 3)
        return res


    def set_opt(self, flag):
        """ERXUDP, ERXTCPのフォーマット設定
            flag: True: ASCII
                  False: Binary
        """
        if user_conf.UDG1WSNE:
            return True
        current = self.get_opt()
        if flag and not current:        # 変更無しの場合はモジュールに書き込まない（FLASHへの書き込み制限）
            self.write(b'WOPT 01\r\n', ['OK 01'])
        elif not flag and current:
            self.write(b'WOPT 00\r\n', ['OK 00'])
        return True


    def get_opt(self):
        """ERXUDP, ERXTCPのフォーマット取得
            retern True: ASCII
                   False: Binary
        """
        if user_conf.UDG1WSNE:
            return True
        res = self.write(b'ROPT\r\n', ['OK'])
        return True if res[0]['MESSAGE'][0] == '01' else False


    def set_echoback_off(self):
        """エコーバックを停止"""
        self.write(b'SKSREG SFE 0\r\n', ['OK'], ignore = True)


    def set_channel(self, ch):
        """Wi-SUNチャンネル設定(保存される)"""
        current = self.get_channel()
        if current != ch:
            bc = '{:02X}'.format(ch).encode()
            self.write(b'SKSREG S02 ' + bc + b'\r\n', ['OK'])


    def get_channel(self):
        """Wi-SUNチャンネル取得
           ESREG イベントで通知を受ける
        """
        res = self.write(b'SKSREG S02' + b'\r\n', ['ESREG', 'OK'], True, 3)
        if res:
            return res[0]['VAL']
        else:
            return False


    def set_pairing_id(self, pairid):
        """ペアリングID設定"""
        self.write(b'SKSREG S0A ' + pairid.encode() + b'\r\n', ['OK'])


    def set_pan_id(self, pan):
        """PAN ID設定(保存される)"""
        current = self.get_pan_id()
        if current != pan:
            bp = '{:04X}'.format(pan).encode()
            self.write(b'SKSREG S03 ' + bp + b'\r\n', ['OK'])


    def get_pan_id(self):
        """PAN ID設定
           ESREG イベントで通知を受ける
        """
        res = self.write(b'SKSREG S03' + b'\r\n', ['ESREG', 'OK'], True, 3)
        if res:
            return res[0]['VAL']
        else:
            return False


    def set_accept_beacon(self, flag):
        """ビーコンリクエストへの反応
            flag True:  応答する
                 False: 応答しない
        """
        bf = b'1' if flag else b'0'
        self.write(b'SKSREG S15 ' + bf + b'\r\n', ['OK'])


    def get_tx_limit(self):
        """送信制限フラグ取得"""
        res = self.write(b'SKSREG SFB\r\n' , ['ESREG', 'OK'], True, 3)
        result = True if res[0]['VAL'] == '1' else False
        return result


    def set_icmp_ctrl(self, flag):
        """ICMP メッセージ処理制御"""
        bf = b'1' if flag else b'0'
        res = self.write(b'SKSREG SA1 ' + bf + b'\r\n', ['OK'])


    def set_password(self, password):
        """パスワード設定"""
        length = len(password)
        if length < 1 or length > 32:
            result = False
        else:
            bp = '{:X} {}'.format(length, password).encode()
            self.write(b'SKSETPWD ' + bp + b'\r\n', ['OK'])
            result = True
        return result


    def set_routeb_id(self, rbid):
        """ルートB ID設定"""
        if len(rbid) != 32:
            result = False
        else:
            self.write(b'SKSETRBID ' + rbid.encode() + b'\r\n', ['OK'])
            result = True
        return result


    def start_paa(self):
        """PAA開始"""
        self.write(b'SKSTART\r\n', ['OK'], True, 10)


    def start_pac(self, ip6):
        """PaC開始"""
        res = self.write(b'SKJOIN ' + ip6.encode() + b'\r\n', [['EVENT 24', 'EVENT 25', 'FAIL']], 
                         ignore = True, timeout = 30)
        if not res:            # time out
            return False
        try:
            result = True if res[0]['COMMAND'] == 'EVENT 25' else False
            return result
        except:     # IndexErrorが発生するときのための暫定処理。要検討
            return False


    def restart_pac(self):
        """PaCをリスタート"""
        res = self.write(b'SKREJOIN\r\n', [['EVENT 24', 'EVENT 25', 'FAIL']], ignore = True, timeout = 30)
        if not res:            # time out
            return False
        try:
            result = True if res[0]['COMMAND'] == 'EVENT 25' else False
            return result
        except:     # IndexErrorが発生するときのための暫定処理。要検討
            return False


    def pac_terminate(self):
        """PANAセッションを終了する"""
        res = self.write(b'SKTERM\r\n', [['OK', 'FAIL', 'EVENT 27', 'EVENT 28']], ignore = True, timeout = 10)
        if not res:            # time out
            return False
        if res[0]['COMMAND'] == 'OK':
            return True
        else:
            return False


    def get_ip6(self, add):
        """IP6アドレス習得"""
        res = self.write(b'SKLL64 ' + add.encode() + b'\r\n', ['UNKNOWN'])
        if not res:            # time out
            return False
        return res[0]['MESSAGE'][0]


    def tcp_connect(self, ip6, rport, lport):
        """TCPコネクション開始"""
        br = ' {:04X}'.format(rport).encode()
        bl = ' {:04X}'.format(lport).encode()
        res = self.write(b'SKCONNECT ' + ip6.encode() + br + bl + b'\r\n', ['ETCP'])
        return res[0]


    def tcp_disconnect(self, handle):
        """TCPコネクション停止"""
        res = self.write(b'SKCLOSE ' + str(handle).encode() + b'\r\n', ['ETCP'])
        return res[0]['STATUS'] == 3


    def tcp_send(self, handle, message):
        """TCPで送信"""
        len_bt =' {:04X} '.format(len(message)).encode()         
        if user_conf.UDG1WSNE:
            res = self.write(b'SKSEND ' + str(handle).encode() + len_bt + ' 0' + message, ['ETCP'])    # for UDG-1-WSNE
        else:
            res = self.write(b'SKSEND ' + str(handle).encode() + len_bt + message, ['ETCP'])
        return res[0]['STATUS'] == 5


    def udp_send(self, handle, ip6, security, port, message):
        """UDPで送信"""
        sec_bt = b' 1' if security else b' 0'
        len_bt = ' {:04X} '.format(len(message)).encode()
        port_bt = ' {:04X}'.format(port).encode()
        if user_conf.UDG1WSNE:
            res = self.write(b'SKSENDTO ' + str(handle).encode() + b' ' + ip6.encode() + port_bt + 
                         sec_bt + b' 0'+ len_bt + message, ['EVENT 21', 'OK'])                  # for UDG-1-WSNE
        else:
            res = self.write(b'SKSENDTO ' + str(handle).encode() + b' ' + ip6.encode() + port_bt + 
                         sec_bt + len_bt + message, ['EVENT 21', 'OK'])

        if not res:               # time out
            self.clear_sk()
            return False

        result = False
        for i in range(len(res)):
            if res[i]['COMMAND'] == 'EVENT 21':
                if res[i]['PARAM'] == '00':     # 送信成功
                    result = True
                    break
                elif res[i]['PARAM'] == '01':   # 送信失敗
                    sys.stdout.write('[Error]: UDP transmission.\n')
                    #if self.get_tx_limit():
                    #    sys.stdout.write('[Error]: TX limit.\n')
                    break
        
        return result


    def ed_scan(self, duration = 4):
        """EDスキャン"""
        bd = '{:X}'.format(duration).encode()
        if user_conf.UDG1WSNE:
            self.write(b'SKSCAN 0 FFFFFFFF ' + bd + b' 0' + b'\r\n', [['EEDSCAN'], ['OK']], ignore = False, timeout = 40)    # for UDG-1-WSNE
        else:
            self.write(b'SKSCAN 0 FFFFFFFF ' + bd + b'\r\n', [['EEDSCAN'], ['OK']], ignore = False, timeout = 40)
        res = []
        while True:
            if self.get_queue_size():
                msg = self.dequeue_message()
                res = msg['MESSAGE']
                break
            else:
                time.sleep(0.01)

        lqi_list = []
        for i in range(0, len(res), 2):
            lqi_list.append([int(res[i + 1], base=16), int(res[i], base=16)])  # [[LQI, channel], [LQI, channel],....]
            lqi_list.sort()  # LQIでソート
        return [lqi_list[0][1], lqi_list[0][0]]  # LQI最小チャンネル [channel, LQImin]


    def active_scan(self, duration = 6):
        """アクティブスキャン"""
        bd = '{:X}'.format(duration).encode()
        if user_conf.UDG1WSNE:
            res = self.write(b'SKSCAN 2 FFFFFFFF ' + bd + b' 0' + b'\r\n', ['EVENT 22'], False, 60)    # for UDG-1-WSNE
        else:
            res = self.write(b'SKSCAN 2 FFFFFFFF ' + bd + b'\r\n', ['EVENT 22'], False, 40)
        
        """ res ← search[found_word_list] """

        if res == False:
            return False

        scan_end = False
        channel_list = []
        channel = {}
        for msg_list in res:
            if msg_list['COMMAND'] == 'EVENT 20':
                pass    # beacon 受信
            elif msg_list['COMMAND'] == 'EPANDESC':
                channel = {}
            elif msg_list['COMMAND'] == 'ACTIVESCAN':
                if 'Channel' in msg_list:
                    channel['Channel'] = msg_list['Channel']
                elif 'Channel Page' in msg_list:
                    channel['Channel Page'] = msg_list['Channel Page']
                elif 'Pan ID' in msg_list:
                    channel['Pan ID'] = msg_list['Pan ID']
                elif 'Addr' in msg_list:
                    channel['Addr'] = msg_list['Addr']
                elif 'LQI' in msg_list:
                    channel['LQI'] = msg_list['LQI']
                elif 'PairID' in msg_list:
                    channel['PairID'] = msg_list['PairID']
                    channel_list.append(channel)
            elif msg_list['COMMAND'] == 'EVENT 22':
                return channel_list
        return False


    @staticmethod
    def parse_message(msg):
        """受信メッセージのパーサー"""
        msg_list = {}

        if msg.startswith('Channel Page'):
            msg_list['COMMAND'] = 'ACTIVESCAN'
            cols = msg.split(':')
            msg_list['Channel Page'] = int(cols[1], base=16)
            return msg_list

        if msg.startswith('Channel'):
            msg_list['COMMAND'] = 'ACTIVESCAN'
            cols = msg.split(':')
            msg_list['Channel'] = int(cols[1], base=16)
            return msg_list

        if msg.startswith('Pan ID'):
            msg_list['COMMAND'] = 'ACTIVESCAN'
            cols = msg.split(':')
            msg_list['Pan ID'] = int(cols[1], base=16)
            return msg_list

        if msg.startswith('Addr'):
            msg_list['COMMAND'] = 'ACTIVESCAN'
            cols = msg.split(':')
            msg_list['Addr'] = cols[1]
            return msg_list

        if msg.startswith('LQI'):
            msg_list['COMMAND'] = 'ACTIVESCAN'
            cols = msg.split(':')
            msg_list['LQI'] = int(cols[1], base=16)
            return msg_list

        if msg.startswith('PairID'):
            msg_list['COMMAND'] = 'ACTIVESCAN'
            cols = msg.split(':')
            msg_list['PairID'] = cols[1]
            return msg_list

        cols = msg.split()

        if cols[0] == 'OK':
            msg_list['COMMAND'] = cols[0]
            if len(cols) > 1:
                msg_list['MESSAGE'] = cols[1:len(cols)]
            return msg_list

        if cols[0] == 'EVENT':
            msg_list['COMMAND'] = cols[0] + ' ' + cols[1]
            msg_list['SENDER'] = cols[2]
            if user_conf.UDG1WSNE:
                if len(cols) == 5:                    # UDG-1-WSNE
                    msg_list['PARAM'] = cols[4]       # UDG-1-WSNE
            else:
                if len(cols) == 4:
                    msg_list['PARAM'] = cols[3]

            return msg_list

        if cols[0] == 'ERXUDP':  # UDP
            msg_list['COMMAND'] = cols[0]
            msg_list['SENDER'] = cols[1]
            msg_list['DEST'] = cols[2]
            msg_list['RPORT'] = int(cols[3], base=16)
            msg_list['LPORT'] = int(cols[4], base=16)
            msg_list['SENDERLLA'] = cols[5]
            msg_list['SECURED'] = int(cols[6], base=16)
            if user_conf.UDG1WSNE:
                msg_list['DATALEN'] = int(cols[8], base=16)
                msg_list['DATA'] = cols[9]
            else:
                msg_list['DATALEN'] = int(cols[7], base=16)
                msg_list['DATA'] = cols[8]

            return msg_list

        if cols[0] == 'ERXTCP':
            msg_list['COMMAND'] = cols[0]
            msg_list['SENDER'] = cols[1]
            msg_list['RPORT'] = int(cols[2], base=16)
            msg_list['LPORT'] = int(cols[3], base=16)
            msg_list['DATALEN'] = int(cols[4], base=16)
            msg_list['DATA'] = cols[5]
            return msg_list

        if cols[0] == 'ETCP':
            msg_list['COMMAND'] = cols[0]
            msg_list['STATUS'] = int(cols[1], base=16)
            msg_list['HANDLE'] = int(cols[2], base=16)
            if msg_list['STATUS'] == 1:
                msg_list['IPADDR'] = cols[3]
                msg_list['RPORT'] = int(cols[4], base=16)
                msg_list['LPORT'] = int(cols[5], base=16)
            return msg_list

        if cols[0] == 'ESREG':
            msg_list['COMMAND'] = cols[0]
            msg_list['VAL'] = cols[1]
            return msg_list

        if cols[0] == 'EPANDESC':
            msg_list['COMMAND'] = 'EPANDESC'
            return msg_list

        if cols[0] == 'EEDSCAN':
            msg_list['COMMAND'] = 'EEDSCAN'
            return msg_list

        if cols[0] == 'EPONG':
            msg_list['COMMAND'] = cols[0]
            if user_conf.UDG1WSNE:
                msg_list['SENDER'] = cols[2]
            else:
                msg_list['SENDER'] = cols[1]
            return msg_list

        if cols[0] == 'EVER':
            msg_list['COMMAND'] = cols[0]
            msg_list['VERSION'] = cols[1]
            return msg_list

        if cols[0] == 'FAIL':
            msg_list['COMMAND'] = cols[0]
            msg_list['ERRCODE'] = cols[1]
            return msg_list

        if cols[0] == 'EINFO':
            msg_list['IPADDR'] = cols[0]
            msg_list['ADDR64'] = cols[1]
            msg_list['CANNEL'] = cols[2]
            msg_list['PANID'] = cols[3]
            msg_list['ADDR16'] = cols[4]
            return msg_list

        # EADDR [IPADDR]
        # ENEIGHBOR [IPADDR ADDR64 ADDR16]
        # EPORT
        #
        # ローカルエコー停止前のローカルエコー対策: 'SKSREG SFE 0'
        if cols[0] == 'SKSREG':
            msg_list['COMMAND'] = 'SKSREG'
            msg_list['REG'] = cols[1]
            msg_list['VAL'] = cols[2]
            return msg_list

        # その他
        msg_list['COMMAND'] = 'UNKNOWN'  # unknown message
        msg_list['MESSAGE'] = cols
        #pprint(msg_list)    # debug
        return msg_list


    def enqueue_message(self, msg_list):
        """メッセージをリストに追加"""
        self.msg_list_lock.acquire()
        self.msg_list_queue.append(msg_list)
        self.msg_list_lock.release()


    def dequeue_message(self):
        """メッセージをリストから取り出す"""
        self.msg_list_lock.acquire()
        
        if self.msg_list_queue:
            result = self.msg_list_queue.pop(0)
        else:
            result = False
            
        self.msg_list_lock.release()
        
        return result
            

    def get_queue_size(self):
        """リスト内のメッセージ数"""
        return len(self.msg_list_queue)


    def uart_open(self, dev, baud, timeout):
        """UARTオープン"""
        try:
            self.uart_hdl = serial.Serial(dev, baud, timeout=timeout)
            self.uart_dev = dev
            self.uart_baud = baud
            return True
        except OSError as msg:
            sys.stdout.write('[Error]: {} @uart_open\n'.format(msg))
            return False


    def uart_close(self):
        """UARTクローズ"""
        try:
            self.uart_hdl.close()
        except OSError as msg:
            sys.stdout.write('[Error]: {} @uart_close\n'.format(msg))


    def write(self, send_msg, search_words = [], ignore = False, timeout = 10):
        """UART書き込み & 受信待ち
            send_msg: 送信データ: bytes
            search_word: 受信待ちコマンド
                (例) ['word1', 'word2', ['word31', 'word32']]: 'word1 -> 'word2' -> 'word31' or 'word32'
            ignore: 途中の受信データを無視する
            timeout: タイムアウト時間[s]
        """
        try:
            #sys.stdout.write('[SND]: {}, timeout = {}\n'.format(send_msg, timeout))    # debug
            self.search['ignore_intermidiate'] = ignore
            self.search['start_time'] = time.time()
            self.search['found_word_list'] = []
            self.search['timeout'] = timeout
            
            self.uart_hdl.write(send_msg)
            self.search['search_words'] = search_words    # run()で監視しているので、一番最後に設定する

            if search_words == []:
                return
            
            while self.search['search_words'] != []:      # run()でself.search['search_words']をpop(0)してゆく
                time.sleep(0.01)
                if time.time() - self.search['start_time'] > self.search['timeout']:
                    self.msg_list_lock.acquire()
                    self.search['search_words'] = []
                    self.msg_list_lock.release()
                    sys.stdout.write('[Error]: Time out. @write\n')   # debug
                    return False
            return self.search['found_word_list']

        except OSError as msg:
            sys.stdout.write('[Error]: {} @write\n'.format(msg))
            self.msg_list_lock.acquire()
            self.search['search_words'] = []
            self.msg_list_lock.release()
            return False


    def read(self):
        """1行読み込み（文字列)"""
        try:
            res = self.uart_hdl.readline().decode().strip()
            #if res: print('read:'+res)   # debug
            return res
        except OSError as msg:
            sys.stdout.write('[Error]: {} @read\n'.format(msg))
            return False


    '''
    def run(self):
        """UART受信用スレッド"""
        while not self.term_flag:
            msg = self.read()
            if msg:
                msg_list = self.parse_message(msg)

                # debug: UDP(PANA)の受信
                if msg_list['COMMAND'] == 'ERXUDP' and msg_list['LPORT'] == self.Y3_UDP_PANA_PORT:
                    #sys.stdout.write('[Note]: PANA message received.\n')
                    pass
               
                elif self.search['search_words']:     # サーチ中である
                    # サーチワードを受信した。
                    search_words = self.search['search_words'][0]                    
                    if isinstance(search_words, list):
                        for word in search_words:
                            if msg_list['COMMAND'].startswith(word):
                                self.search['found_word_list'].append(msg_list)
                                self.search['search_words'].pop(0)
                                break
                    elif msg_list['COMMAND'].startswith(search_words):
                        self.search['found_word_list'].append(msg_list)
                        self.search['search_words'].pop(0)
                    
                    elif self.search['ignore_intermidiate']:
                        pass    # 途中の受信データを破棄 
                
                    else:    # サーチワードではなかった
                        self.enqueue_message(msg_list)
                
                else:   # サーチ中ではない
                    self.enqueue_message(msg_list)
                
            elif self.search['timeout']:  # read()がタイムアウト，write()でタイムアウトが設定されている
                if time.time() - self.search['start_time'] > self.search['timeout']:
                    self.search['found_word_list'] = []
                    self.search['search_words'] = []
                    self.search['timeout'] = 0
    '''


    def terminate(self):
        """run()の停止"""
        self.term_flag = True
        self.join()
