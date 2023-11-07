using System;
using System.IO;
using System.Diagnostics;
using System.Windows;

using Newtonsoft.Json;

//Json 형식으로 파일을 저장하거나 읽어올 수 있음.
//List를 가진 Class를 만들어야 파싱이 가능.
public class JsonFileIO
{
    //T: 제네릭(어떤 구조든 대입가능)
    //파일 불러오기
    public static T Load<T>(string fileName)
    {
        string str;

        str = JsonFileIO.ReadStringFromFile(fileName);

        if (str != null)
        {
            Debug.WriteLine("Load Json!");
            //return JsonUtility.FromJson<T>(str);
            return JsonConvert.DeserializeObject<T>(str);
        }

        return default(T);
    }

    //파일 저장하기
    public static void Save(object obj, string fileName)
    {
        Debug.WriteLine("Save Json!");
        //Debug.Write(JsonUtility.ToJson(obj, prettyPrint: true));
        //string strSave = JsonUtility.ToJson(obj, prettyPrint: true);
        Debug.Write(JsonConvert.SerializeObject(obj));
        string strSave = JsonConvert.SerializeObject(obj);
        JsonFileIO.WriteStringToFile(strSave, fileName);
    }

    private static void WriteStringToFile(string str, string filename)
    {
#if !WEB_BUILD
        string path = PathForDocumentsFile(filename);
        FileStream file = new FileStream(path, FileMode.Create, FileAccess.Write);

        StreamWriter sw = new StreamWriter(file);
        sw.WriteLine(str);

        sw.Close();
        file.Close();
#endif
    }

    private static string ReadStringFromFile(string fileName)//, int lineIndex )
    {
#if !WEB_BUILD
        string path = PathForDocumentsFile(fileName);

        if (File.Exists(path))
        {
            FileStream file = new FileStream(path, FileMode.Open, FileAccess.Read);
            StreamReader st = new StreamReader(file);

            string str = null;
            string strResult = "";
            str = st.ReadLine();

            while (strResult != null)
            {
                strResult = st.ReadLine();
                str = str + "\n" + strResult;
            }

            st.Close();
            file.Close();

            return str;
        }
        else
        {
            return null;
        }
#else
return null;
#endif
    }

    private static string PathForDocumentsFile(string fileName)
    {
        string path = Environment.CurrentDirectory;
        //        path = path.Substring(0, path.LastIndexOf('/'));
        path = path.Substring(path.LastIndexOf('/') + 1);
        return Path.Combine(path, fileName);
    }
}
