# Image Comparison Script

The [image_comparison.py](../../Software/Testing/Image-Compare/image_comparison.py) script provides functionality to compare multiple images against a ground truth image using various image comparison algorithms and calculate a weighted overall score for each image based on the results.

## Table of Contents
- [Functions](#functions)
    - [calculate_overall_score](#calculate_overall_score)
    - [normalize](#normalize)
    - [calculate_scores](#calculate_scores)
    - [get_image_comparison_algorithms](#get_image_comparison_algorithms)
    - [get_images](#get_images)
    - [run_image_comparison_algorithms](#run_image_comparison_algorithms)
    - [save_csv_results](#save_csv_results)
    - [main](#main)
- [Usage](#usage)
- [Example](#example)

## Functions

### `calculate_overall_score(image_data, weights)`
Calculates the overall score for an image based on individual metric scores and weights.

### `normalize(value, min_val, max_val)`
Normalizes a given value to a range between 0 and 1.

### `calculate_scores(results_dict, weights)`
Calculates overall scores for all images in the results dictionary.

### `get_image_comparison_algorithms(algorithms_dict)`
Dynamically imports image comparison algorithms based on a dictionary mapping.

### `get_images(input_folder)`
Retrieves a list of image files from a specified folder.

### `run_image_comparison_algorithms(input_folder, input_images, gt_image, image_comparison_algorithms)`
Runs specified image comparison algorithms on each input image against the ground truth image.

### `save_csv_results(results_dict, output)`
Saves the image comparison results to a CSV file.

### `main(input_folder, ground_truth, output_folder, weights, algorithms_dict)`
Main function coordinating the image comparison and evaluation process.

## Usage

1. Ensure the image comparison algorithm modules are located in the `algorithms` directory.
2. Modify the `input_folder`, `ground_truth`, `output_folder`, `weights`, and `algorithms_dict` variables in the main block to match your setup.
3. Run the script. The results will be saved in a CSV file in the `output` folder.

## Example
```python
if __name__ == '__main__':
    algorithms_dict = {'ncc': 'normalized_cross_correlation', 'ssim': 'ssim', 'mse': 'mean_square_error'}
    input_folder = 'img/full_lab_significant/'
    ground_truth = 'img/ground_truth/ground_truth.png'
    output_folder = 'output/'
    weights = {'ncc': 0.45, 'ssim': 0.35, 'mse': 0.20}

    main(input_folder, ground_truth, output_folder, weights, algorithms_dict)
```


# Creating Graphs

The [make_graphs.py](../../Software/Testing/make_graphs.py) script provides functionality to visualize and analyze image comparison results from a CSV file. It generates various plots and statistical summaries to gain insights into the data.

## Table of Contents
- [Functions](#functions)
    - [categorize_image](#categorize_image)
    - [plot_histograms](#plot_histograms)
    - [plot_scatter_plots](#plot_scatter_plots)
    - [print_correlation_matrix](#print_correlation_matrix)
    - [print_statistical_summary](#print_statistical_summary)
    - [print_grouped_means](#print_grouped_means)
    - [plot_average_scores](#plot_average_scores)
    - [plot_with_annotations](#plot_with_annotations)
    - [create_heatmap](#create_heatmap)
    - [scatter_plot_by_group](#scatter_plot_by_group)
    - [histogram_by_group](#histogram_by_group)
- [Usage](#usage)

## Functions

### `categorize_image(name)`
Categorizes images based on their names (e.g., "Upp", "Std", "Others").

### `plot_histograms(data)`
Plots histograms for each image analysis metric (NCC, SSIM, MSE, Overall Score).

### `plot_scatter_plots(data)`
Plots scatter plots to visualize relationships between image analysis metrics.

### `print_correlation_matrix(data)`
Calculates and prints the correlation matrix of image analysis metrics.

### `print_statistical_summary(data)`
Calculates and prints descriptive statistics for image analysis metrics.

### `print_grouped_means(data)`
Calculates and prints mean values of metrics grouped by image categories.

### `plot_average_scores(data)`
Plots average overall scores for each image category.

### `plot_with_annotations(data)`
Creates a scatter plot with image names annotated on the points.

### `create_heatmap(data)`
(Not implemented) Function to create a heatmap.

### `scatter_plot_by_group(data)`
Creates a scatter plot of overall scores, colored by image category.

### `histogram_by_group(data)`
Creates histograms of overall scores, grouped by image category.

## Usage

1. Make sure the CSV file containing image comparison results (`output/result.csv` as an example) is present in the `output` directory.
2. Modify the file path if necessary.
3. Run the script. The script will generate various plots and statistics based on the data and add them to the output folder.


