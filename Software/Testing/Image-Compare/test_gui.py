""" Deprecated code """

import tkinter as tk
from tkinter import filedialog
import os
import importlib
import csv
import threading


# Functions from your provided code
def calculate_overall_score(image_data, weights):
    score = 0
    for metric, weight in weights.items():
        if "mse" in metric:
            score += (1 - image_data[metric]) * weight
        else:
            score += image_data[metric] * weight
    return score

def calculate_scores(results_dict, weights):
    for img in results_dict:
        overall_score = calculate_overall_score(results_dict[img], weights)
        results_dict[img]['overall_score'] = overall_score
    return results_dict

def get_image_comparison_algorithms(algorithms_dict):
    algorithms = {}
    for short_name, full_name in algorithms_dict.items():
        module = importlib.import_module(f"algorithms.{full_name}")
        algorithms[short_name] = getattr(module, f"main")
    return algorithms

def get_images(input_folder):
    return os.listdir(input_folder)

def run_image_comparison_algorithms(input_folder, input_images, gt_image, image_comparison_algorithms):
    results = {}
    threads = []
    lock = threading.Lock()  # Lock for safely updating the results dictionary

    def process_image_subset(images_subset):
        subset_results = {}
        for image in images_subset:
            image_name = str(image).replace(".png", "")
            subset_results[image_name] = {}
            for algorithm_name, algorithm_func in image_comparison_algorithms.items():
                subset_results[image_name][algorithm_name] = algorithm_func(input_folder + image, gt_image)
        
        with lock:
            results.update(subset_results)

    # Splitting the images into chunks and creating threads
    num_threads = 4  # You can adjust the number of threads based on your system capabilities
    chunk_size = len(input_images) // num_threads
    for i in range(num_threads):
        start_index = i * chunk_size
        end_index = start_index + chunk_size if i < num_threads - 1 else len(input_images)
        thread = threading.Thread(target=process_image_subset, args=(input_images[start_index:end_index],))
        threads.append(thread)
        thread.start()

    # Waiting for all threads to finish
    for thread in threads:
        thread.join()

    return results

def save_csv_results(results_dict, output):
    with open(f'{output}results_gui.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Image", "NCC", "SSIM", "MSE", "Overall Score"])
        for img, values in results_dict.items():
            img = os.path.basename(img)
            writer.writerow([img, values["ncc"], values["ssim"], values["mse"], values["overall_score"]])

def main(input_folder, ground_truth, output_folder, weights, algorithms_dict):
    input_images = get_images(input_folder)
    image_comparison_algorithms = get_image_comparison_algorithms(algorithms_dict)
    results = run_image_comparison_algorithms(input_folder, input_images, ground_truth, image_comparison_algorithms)
    results_ = calculate_scores(results, weights)
    save_csv_results(results_, output_folder)

# GUI Application
class ImageComparisonApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Image Comparison Tool")
        self.geometry("600x400")

        # Widgets
        self.create_widgets()

    def create_widgets(self):
        tk.Label(self, text="Input Folder:").grid(row=0, column=0)
        self.input_folder = tk.Entry(self, width=50)
        self.input_folder.grid(row=0, column=1)
        self.input_folder.insert(tk.END, "/img/full_lab/")

        tk.Label(self, text="Ground Truth Image:").grid(row=1, column=0)
        self.ground_truth = tk.Entry(self, width=50)
        self.ground_truth.grid(row=1, column=1)
        self.ground_truth.insert(tk.END, "/img/ground_truth/ground_truth.png")

        tk.Label(self, text="Output Folder:").grid(row=2, column=0)
        self.output_folder = tk.Entry(self, width=50)
        self.output_folder.grid(row=2, column=1)
        self.output_folder.insert(tk.END, "output/")

        tk.Label(self, text="Weights (NCC, SSIM, MSE):").grid(row=3, column=0)
        self.weights_ncc = tk.Entry(self, width=10)
        self.weights_ncc.grid(row=3, column=1, sticky="w")
        self.weights_ssim = tk.Entry(self, width=10)
        self.weights_ssim.grid(row=3, column=1)
        self.weights_mse = tk.Entry(self, width=10)
        self.weights_mse.grid(row=3, column=1, sticky="e")

        self.run_button = tk.Button(self, text="Run", command=self.run_analysis)
        self.run_button.grid(row=4, column=1, pady=10)

        self.result_text = tk.Text(self, height=10, width=75)
        self.result_text.grid(row=5, column=0, columnspan=2, padx=10, pady=10)

    def run_analysis(self):
        self.result_text.insert(tk.END, "Starting analysis...\n")
        # Create a new thread to handle the long-running operation
        thread = threading.Thread(target=self.process_images)
        thread.start()

    def process_images(self):
        input_folder = self.input_folder.get()
        ground_truth = self.ground_truth.get()
        output_folder = self.output_folder.get()
        weights = {
            'ncc': float(self.weights_ncc.get()),
            'ssim': float(self.weights_ssim.get()),
            'mse': float(self.weights_mse.get())
        }
        algorithms_dict = {'ncc': 'normalized_cross_correlation', 'ssim': 'ssim', 'mse': 'mean_square_error'}

        try:
            main(input_folder, ground_truth, output_folder, weights, algorithms_dict)
            self.result_text.insert(tk.END, "Analysis completed successfully. Check output folder for results.\n")
        except Exception as e:
            self.result_text.insert(tk.END, f"An error occurred: {e}\n")

if __name__ == "__main__":
    app = ImageComparisonApp()
    app.mainloop()
