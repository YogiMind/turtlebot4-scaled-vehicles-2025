import os
import importlib
import csv

def calculate_overall_score(image_data, weights):
    score = 0
    for metric, weight in weights.items():
        if "mse" in metric:
            score += (1 - image_data[metric]) * weight
        else:
            score += image_data[metric] * weight
    return score

def normalize(value, min_val, max_val):
    return (value - min_val) / (max_val - min_val)

def calculate_scores(results_dict, weights):
    for img in results_dict:
        overall_score = calculate_overall_score(results_dict[img], weights)
        results_dict[img]['overall_score'] = overall_score
        print(f"Image: {img}, Overall Score: {overall_score}")
    return results_dict

def get_image_comparison_algorithms(algorithms_dict):
    algorithms = {} 
    for short_name, full_name in algorithms_dict.items():
        module = importlib.import_module(f"algorithms.{full_name}")
        algorithms[short_name] = getattr(module, f"main")
    return algorithms

def get_images(input_folder):
    # Get the list of images in the input folder
    return os.listdir(input_folder)

def run_image_comparison_algorithms(input_folder, input_images, gt_image, image_comparison_algorithms):
    results = {}
    for image in input_images:
        if ".png" not in image:
            continue
        else:
            image_name = str(image).replace(".png", "")
            results[image_name] = {}
            for algorithm_name, algorithm_func in image_comparison_algorithms.items():
                results[image_name][algorithm_name] = algorithm_func(input_folder + image, gt_image)
    return results

def save_csv_results(results_dict, output):
    with open(f'{output}results.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Image", "NCC", "SSIM", "MSE", "Overall Score"])
        for img, values in results_dict.items():
            print(img, values)
            # Remove the path from the image name
            img = os.path.basename(img)
            writer.writerow([img, values["ncc"], values["ssim"], values["mse"], values["overall_score"]])

def main(input_folder, ground_truth, output_folder, weights, algorithms_dict):
    input_images = get_images(input_folder)

    image_comparison_algorithms = get_image_comparison_algorithms(algorithms_dict)

    results = run_image_comparison_algorithms(input_folder, input_images, ground_truth, image_comparison_algorithms)

    results_ = calculate_scores(results, weights)

    save_csv_results(results_, output_folder)

if __name__ == '__main__':
    # Algorithms used for image comparisons
    algorithms_dict = {'ncc': 'normalized_cross_correlation', 'ssim': 'ssim', 'mse': 'mean_square_error'}
    input_folder = 'img/full_lab_significant/'
    # Image that is used to compare the input images to. 
    ground_truth = 'img/ground_truth/ground_truth.png'
    output_folder = 'output/'
    weights = {'ncc': 0.45, 'ssim': 0.35, 'mse': 0.20}

    main(input_folder, ground_truth, output_folder, weights, algorithms_dict)


