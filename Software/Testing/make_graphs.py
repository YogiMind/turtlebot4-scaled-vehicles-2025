import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns


# Define a function to categorize images based on their names
def categorize_image(name):
    if "Upp" in name:
        return "Upp"
    elif "Std" in name:
        return "Std"
    else:
        return "Others"

# Apply the categorization
data['Category'] = data['Image'].apply(categorize_image)

# Plotting histograms of each metric
def plot_histograms(data):
    sns.set(style="whitegrid")
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle('Distribution of Image Analysis Metrics', fontsize=16)

    metrics = ['NCC', 'SSIM', 'MSE', 'Overall Score']
    colors = ['skyblue', 'lightgreen', 'salmon', 'gold', 'lavender']
    for ax, metric, color in zip(axes.flat, metrics, colors):
        sns.histplot(data[metric], bins=10, kde=True, ax=ax, color=color)
        ax.set_title(f'Histogram of {metric}')
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    fig.delaxes(axes.flatten()[5])  # Remove the last empty subplot
    plt.savefig(f'histograms.png')    
    plt.show()


# Plotting scatter plots to explore relationships between metrics
def plot_scatter_plots(data):
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('Relationships Between Image Analysis Metrics', fontsize=16)
    
    sns.scatterplot(x='SSIM', y='NCC', data=data, ax=axes[0, 0], color='blue')
    axes[0, 0].set_title('SSIM vs NCC')
    
    #sns.scatterplot(x='SSIM', y='SSIM ', data=data, ax=axes[0, 1], color='green')
    #axes[0, 1].set_title('SSIM vs SSIM w Edge')
    
    sns.scatterplot(x='SSIM', y='MSE', data=data, ax=axes[1, 0], color='red')
    axes[1, 0].set_title('MSE vs SSIM')
    
    sns.scatterplot(x='SSIM', y='Overall Score', data=data, ax=axes[1, 1], color='purple')
    axes[1, 1].set_title('Overall Score vs SSIM')
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(f'scatter_plots.png')
    plt.show()


def print_correlation_matrix(data):
    correlation_matrix = data[['NCC', 'SSIM', 'MSE', 'Overall Score']].corr()
    print(correlation_matrix)

    #Save correlation matrix to png
    plt.figure(figsize=(8, 6))
    sns.heatmap(correlation_matrix, annot=True, cmap='coolwarm', center=0)
    plt.title('Correlation Matrix of Image Analysis Metrics')
    plt.savefig(f'correlation_matrix.png')
    plt.show()

def print_statistical_summary(data):
    statistical_summary = data[['NCC', 'SSIM', 'MSE', 'Overall Score']].describe()
    print(statistical_summary)
    # Save statistical summary to csv
    statistical_summary.to_csv('statistical_summary.csv')

# Calculate mean of each metric grouped by the image category
def print_grouped_means(data):
    category_means = data.groupby('Category')[['NCC', 'SSIM', 'MSE', 'Overall Score']].mean()
    print(category_means)

# Calculate the average Overall Score by category and plot
def plot_average_scores(data):
    average_scores = data.groupby('Category')['Overall Score'].mean().reset_index()
    plt.figure(figsize=(8, 5))
    sns.barplot(x='Category', y='Overall Score', data=average_scores, palette='coolwarm')
    plt.title('Average Overall Score by Image Category')
    plt.xlabel('Category')
    plt.ylabel('Average Overall Score')
    plt.ylim(0.55, 0.75)  # Set the limits for better visualization
    plt.savefig(f'average_scores.png')
    plt.show()

# Plot with image names annotated
def plot_with_annotations(data):
    plt.figure(figsize=(10, 6))
    plot = sns.scatterplot(x='SSIM', y='NCC', data=data)
    plt.title('SSIM vs NCC')
    # Annotating each point with the corresponding image name
    for i in range(data.shape[0]):
        plt.text(data.iloc[i]['SSIM'], data.iloc[i]['NCC'], data.iloc[i]['Image'], 
                 fontdict={'size': 9}, ha='right') 
    plt.show()

def create_heatmap(data):
    pass

def scatter_plot_by_group(data):
    # Create scatter plot of overall score colored by category
    plt.figure(figsize=(10, 6))
    sns.scatterplot(x='Overall Score', y='Overall Score', data=data, hue='Category', palette='coolwarm')
    plt.title('Overall Score by Image Category')
    plt.savefig(f'overall_score_by_category.png')

def histogram_by_group(data):
    # Create histogram with Overall scores by category
    plt.figure(figsize=(10, 6))
    sns.histplot(data, x='Overall Score', hue='Category', palette='coolwarm', kde=True, bins=10)
    plt.title('Distribution of Overall Scores by Image Category')
    plt.savefig(f'overall_score_distribution_hist.png')

# Run all analysis functions

if __name__ == '__main__':
    data = pd.read_csv('image_analysis_results.csv')
    plot_histograms(data)
    plot_scatter_plots(data)
    print_correlation_matrix(data)
    print_statistical_summary(data)
    print_grouped_means(data)
    plot_average_scores(data)
    plot_with_annotations(data)
    create_heatmap(data)
    scatter_plot_by_group(data)
    histogram_by_group(data)
