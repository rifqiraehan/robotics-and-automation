using System.Windows;
using CommunityToolkit.Mvvm.DependencyInjection; // Jika gunakan IoC, optional

namespace Sudut
{
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            // DataContext sudah di-set di XAML; jika perlu IoC: DataContext = Ioc.Default.GetService<ServoViewModel>();
        }
    }
}