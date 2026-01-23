from bs4 import BeautifulSoup as soup
import requests

def parse_airfoil_search_pages(html_content):
    """
    Parses the HTML content of an airfoil search results page and extracts
    airfoil names and their corresponding detail page links.

    Args:
        html_content (str): The HTML content of the search results page.
    Returns:
        list of urls of airfoil detail pages.
    """
    page_soup = soup(html_content, 'html.parser')
    airfoil_links = []

    # get table with class afSearchResult
    table = page_soup.find('table', {'class': 'afSearchResult'})
    if not table:
        raise Exception("No airfoil search results table found.")
    
    # for each tr in table at an odd index
    for index, row in enumerate(table.find_all('tr')):
        # row has a td with class cell1 open it and set it equal to airfoil_name_cell
        airfoil_name_cell = row.find('td', {'class': 'cell1'})
        if not airfoil_name_cell:
            continue  # skip rows without the expected structure
        try:
            airfoil_link = airfoil_name_cell.find('a')['href']
        except TypeError:
            raise Exception(f"No link found for airfoil in row \ncols: {row.find_all('td')}")
        full_link = f"http://airfoiltools.com{airfoil_link}"
        airfoil_links.append(full_link) 

    return airfoil_links

def fetch_page_content(url):
    """
    Fetches the HTML content of a given URL.

    Args:
        url (str): The URL to fetch.
    Returns:
        str: The HTML content of the page.
    """
    try:
        response = requests.get(url)
    except Exception as e:
        print(f"ERROR: An error occurred while fetching {url}: {e}")
        return None
    if response.status_code == 200:
        return response.text
    else:
        print(f"ERROR: Failed to fetch {url}: Status code {response.status_code}")
        return None

def get_bat_files(airfoil_links):
    """
    Given a list of airfoil detail page links, fetches each page and extracts
    the link to the corresponding .bat file.

    Args:
        airfoil_links (list): List of URLs to airfoil detail pages.
    Returns:
        list: List of URLs to .bat files.
    """
    bat_file_links = []
    # for eahch airfoil link, open the page and return the link to the .bat file stored in the a tage with text "Source dat file"
    for link in airfoil_links:
        page_content = fetch_page_content(link)
        if page_content is None:
            print(f"Skipping {link} due to fetch error.")
            continue
        page_soup = soup(page_content, 'html.parser')
        bat_file_link = None
        for a_tag in page_soup.find_all('a'):
            if 'Source dat file' in a_tag.text:
                bat_file_link = a_tag['href']
                break
        if bat_file_link:
            bat_file_links.append(bat_file_link)
        else:
            print(f"No .bat file link found on page: {link}")
    return bat_file_links

if __name__ == "__main__":
    page_number = 0
    max_page_number = 9 # for the specifc pasge below
    aerfoils_links = []
    for pn in range(page_number, max_page_number + 1):
        url = f"""http://airfoiltools.com/search/index?m%5BtextSearch%5D=&m%5BmaxCamber%5D=&m%5BminCamber%5D=&m%5BmaxThickness%5D=&m%5BminThickness%5D=20&m%5Bgrp%5D=&m%5Bsort%5D=1&m%5Bpage%5D={pn}&m%5Bcount%5D=94"""
        print(f"Fetching page {pn}: {url}")
        page_content = fetch_page_content(url)
        if page_content is None:
            print(f"Skipping page {pn} due to fetch error.")
            continue
        aerfoils_links.extend(parse_airfoil_search_pages(page_content))
        print(f"Total airfoil links found in pages so far: {len(aerfoils_links)}")

    bat_files = get_bat_files(aerfoils_links)
    # now download each bat file into mechanical\airfoil_performance\scraped_dat
    for bf in bat_files:
        print(f"Downloading .bat file from: {bf}")
        bat_content = fetch_page_content(bf)
        if bat_content is None:
            print(f"Skipping download for {bf} due to fetch error.")
            continue
        file_name = bf.split('/')[-1]
        with open(f"mechanical/airfoil_performance/scraped_dat/{file_name}", 'w') as f:
            f.write(bat_content)
        print(f"Saved .bat file as: mechanical/airfoil_performance/scraped_dat/{file_name}")

    print(f"Total .bat files found: {len(bat_files)}")
