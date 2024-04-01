from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.uix.filechooser import FileChooserListView
from kivy.uix.popup import Popup
import pandas as pd
import matplotlib.pyplot as plt


class MainApp(App):
    def __init__(self, **kwargs):
        super(MainApp, self).__init__(**kwargs)
        self.selected_file = ""

    def build(self):
        layout = BoxLayout(orientation='vertical')
        file_chooser = FileChooserListView()
        file_chooser.bind(on_submit=self.on_file_selected)
        layout.add_widget(file_chooser)
        return layout

    def on_file_selected(self, instance, selection, *args):
        if len(selection) > 0:
            self.selected_file = selection[0]
            self.show_graphs()

    def show_graphs(self):
        try:
            data = pd.read_csv(self.selected_file)
            if 'time' not in data.columns:
                self.show_warning("Ошибка", "Файл не содержит столбец 'time'")
                return

            columns_to_plot = [col for col in data.columns if col != 'time']

            # Построение общего графика
            plt.figure(figsize=(10, 6))
            for col in columns_to_plot:
                plt.plot(data['time'], data[col], label=col)
            plt.xlabel('Время')
            plt.ylabel('Значение')
            plt.title('Общий график')
            plt.legend()
            plt.grid(True)
            plt.show()

            # Построение графиков для каждого столбца относительно времени
            for col in columns_to_plot:
                plt.figure(figsize=(8, 4))
                plt.plot(data['time'], data[col])
                plt.xlabel('Время')
                plt.ylabel(col)
                plt.title(f'График {col}')
                plt.grid(True)
                plt.show()

        except Exception as e:
            self.show_warning("Ошибка", str(e))

    def show_warning(self, title, message):
        content = Button(text=message)
        popup = Popup(title=title, content=content, size_hint=(None, None), size=(400, 200))
        content.bind(on_press=popup.dismiss)
        popup.open()


if __name__ == '__main__':
    MainApp().run()
