using System;
using System.Diagnostics;
using System.IO;
using System.Windows.Forms;
using Timer = System.Windows.Forms.Timer;

namespace MP
{
    class TKVplay
    {
        private int num = 0, cnt = 0;


        int Play_Slider = 0;
        int Play_SlidermaxValue = 0;


        string[,] Joint_data = new string[100, 29];

        string[,] Read_data = new string[100000, 30];
        public string[] input_value = new string[29];
        public float[] objSlider = new float[29];

        private int pre_text_cur_value = 0;

        private int read_num = 0;

        private float Next_action_time = 0.0f; private float Next_frame_time = 0.0f; private float fTickTime;

        public byte State_num = 0;

        private int text_cur_value = 0, next_action_cnt = 0;

        bool frame_error = false, Edit_flag = false;

        bool Toggle_bool = true;

        bool Export_bool = true;

        bool Upload_bool = true;

        private int Action_num = 0;
        private int[] Action_step = new int[100];
        private float correction_value = 0.0f;





        #region //Play by timer __________________________________________________________________________________________
        public int playrate = 100;// TKV재생 프래임레이트 기본 10FPS
        private Timer timer; //TKV재생 타이머 선언
        private int number = 0;//TKV재생 타이머 카운트
        string[] lines; // 파일의 전체 내용을 한 번에 읽어옵니다.
        public int totalLines; //파일 총줄수 중 플레이 데이터 줄수
        public string[] dataArray = new string[29]; // 29개의 값을 저장할 배열
        public int lineIndex = 0;//읽어들일 파일의 줄 순서
        string filePath; //파일 위치(파일명 포함)

        public void timerMain()
        {
            if (number < 1)//play start
            {
                InitializeTimer();
                Debug.WriteLine("==timer.start==============(Single Event)=");
            }
            else if (timer.Enabled)//pause
            {
                timer.Stop();
                Debug.WriteLine("==timer.stop==============(Single Event)=");
            }
            else//Play re start
            {
                timer.Start();
                Debug.WriteLine("==timer.start==============(Single Event)=");
            }

        }

        private void InitializeTimer()
        {
            timer = new Timer();
            timer.Interval = playrate;
            timer.Tick += Timer_Tick;
            timer.Start();
        }


        private void Timer_Tick(object sender, EventArgs e) //재생 타이머
        {
            number++; // 시간숫자 증가
            Debug.WriteLine("=================");
            Debug.WriteLine("TimerCount: " + number);
            Debug.WriteLine("=================");

            try
            {

                Console.WriteLine("파일 총 줄 수/현재줄: " + totalLines + "/" + lineIndex);
                // 파일의 각 줄을  처리
                if (lineIndex < totalLines)
                {
                    string line = lines[lineIndex];

                    if (lineIndex == totalLines - 1)// 마지막 줄에 도달하면 종료
                    {
                        Console.WriteLine("마지막 줄에 도달 정지.");
                        timer.Stop();
                        return;
                    }
                    else
                    {
                        if (lineIndex > 1)
                        {
                            dataArray = line.Split(new char[] { ' ' }, StringSplitOptions.RemoveEmptyEntries); // 띄어쓰기로 나눈 후 배열에 저장

                            // 배열 출력 (각도 분배로직 추가 필요)
                            Console.WriteLine("배열 업데이트: " + string.Join(", ", dataArray));
                        }
                        lineIndex++;
                    }
                }
                else
                {
                    Console.WriteLine("내용이 없습니다. 정지.");
                    timer.Stop();
                }

            }
            catch (Exception)// e)
            {
                //   Console.WriteLine("오류 발생: " + e.Message);
            }


        }

        #endregion //timer test -------------------------------------------------------------------






        public void Upload()
        {
            //TKV file loading
#if UNITY_2017_1_OR_NEWER
            var extensions = new[] {  new ExtensionFilter("Tkv File", "tkv" ), new ExtensionFilter("All Files", "*" ),};

            var paths = StandaloneFileBrowser.OpenFilePanel("Open File", "", extensions, true);
#else
            OpenFileDialog ofd = new OpenFileDialog();
            ofd.Title = "TKV 파일 불러오기";
            ofd.FileName = "*.tkv";
            ofd.Filter = "Tkv File (*.tkv) | *.tkv; | 그림 파일 (*.jpg, *.gif, *.bmp) | *.jpg; *.gif; *.bmp; | 모든 파일 (*.*) | *.*";
            DialogResult dr = ofd.ShowDialog(); //파일 오픈창 로드
            if (dr == DialogResult.OK) //OK버튼 클릭시
            {
                //File명과 확장자를 가지고 온다.
                string fileName = ofd.SafeFileName;
                //File경로와 File명을 모두 가지고 온다.
                string fileFullName = ofd.FileName;
                //File경로만 가지고 온다.
                string filePath = fileFullName.Replace(fileName, "");
            }
            else if (dr == DialogResult.Cancel)  //취소버튼 클릭시 또는 ESC키로 파일창을 종료 했을경우
            {
                return;
            }
#endif
            var paths = filePath = ofd.FileName; //StandaloneFileBrowser.OpenFilePanel("Open File", "", extensions, true);
            if (number > 1) Stop();

#if UNITY_2017_1_OR_NEWER

            string upload_path = string.Join(".", paths);

            string video_upload_path = "file://" + upload_path.Replace("tkv", "mp4");
            video_upload_path = video_upload_path.Replace("\\", "/");

            FileInfo fileInfo = new FileInfo(upload_path);

            read_num = 0;

            string value;

            char sp = ' ';

            if (fileInfo.Exists)
            {
                StreamReader reader = new StreamReader(upload_path);

                while ((value = reader.ReadLine()) != null)
                {

                    string[] Text_raw = value.Split(sp);

                    if (Text_raw[0] == "actionkey")
                    {
                        Action_step[Action_num] = read_num - 1;
                        Debug.Log(Action_step[Action_num]);

                        Action_num++;
                    }
                    else
                    {
                        if (read_num > 0)
                        {
                            for (int i = 0; i < Text_raw.Length; i++)
                                Read_data[read_num - 1, i] = Text_raw[i];
                        }

                        read_num++;
                    }
                }

                reader.Close();
                //Popup.SetActive(true);
            }

            else
            {
                //Popup.SetActive(true);
            }

            Play_SlidermaxValue = read_num - 1;
            State_num = 2;
#else
            try
            {
                lines = File.ReadAllLines(filePath);                // 파일의 전체 내용을 한 번에 읽어옵니다.
                totalLines = lines.Length;                // 파일의 총 줄 수

                Console.WriteLine("파일 총 줄 수/현재줄: " + totalLines + "/" + lineIndex);
                Console.WriteLine("Uplude Finish");
            }
            catch (Exception)// e)
            {
                //   Console.WriteLine("오류 발생: " + e.Message);
            }
#endif
        }
        public void Play()
        {
            timerMain();
            State_num = 1;//재생
        }
        public void Pause()
        {
            timer.Stop();
            Debug.WriteLine("==timer.stop==============(Single Event)=");
            State_num = 2;//일시정지
        }
        public void Stop()
        {
            timer.Stop();
            number = lineIndex = State_num = 0;//정지
        }

    }
}
