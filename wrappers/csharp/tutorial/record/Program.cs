using System;
using System.Diagnostics;
using System.Linq;

namespace Intel.RealSense
{

    class Program
    {
        [STAThread]
        static void Main(string[] args)
        {
            var pipe = new Pipeline();
            using (var ctx = new Context())
            {
                var devices = ctx.QueryDevices();
                var dev = devices[0];

                Console.WriteLine("\nUsing device 0, an {0}", dev.Info[CameraInfo.Name]);
                Console.WriteLine("    Serial number: {0}", dev.Info[CameraInfo.SerialNumber]);
                Console.WriteLine("    Firmware version: {0}", dev.Info[CameraInfo.FirmwareVersion]);

                var sensors = dev.QuerySensors();
                var depthSensor = sensors[0];
                var colorSensor = sensors[1];

                var depthProfile = depthSensor.StreamProfiles
                                    .Where(p => p.Stream == Stream.Depth)
                                    .OrderBy(p => p.Framerate)
                                    .Select(p => p.As<VideoStreamProfile>()).First();

                var colorProfile = colorSensor.StreamProfiles
                                    .Where(p => p.Stream == Stream.Color)
                                    .OrderBy(p => p.Framerate)
                                    .Select(p => p.As<VideoStreamProfile>()).First();

                var cfg = new Config();
                cfg.EnableStream(Stream.Depth, depthProfile.Width, depthProfile.Height, depthProfile.Format, depthProfile.Framerate);
                cfg.EnableStream(Stream.Color, colorProfile.Width, colorProfile.Height, colorProfile.Format, colorProfile.Framerate);
                cfg.EnableRecordToFile("csharp_rosbag.bag");

                var pp = pipe.Start(cfg);
            }

            var stopwatch = new Stopwatch();
            var elapse_time = stopwatch.ElapsedMilliseconds;
            stopwatch.Start();

            while (elapse_time <= 5000) // record  5 seconds of footage
            {
                elapse_time = stopwatch.ElapsedMilliseconds;
                using (var frames = pipe.WaitForFrames())
                using (var depth = frames.DepthFrame)
                {
                    
                    Console.WriteLine("[" + elapse_time / 1000 + " sec] The camera is pointing at an object " +
                        depth.GetDistance(depth.Width / 2, depth.Height / 2) + " meters away\t");

                    Console.SetCursorPosition(0, 0);
                }
            }
            stopwatch.Stop();
            pipe.Stop();
            pipe.Dispose();            
        }
    }
}
