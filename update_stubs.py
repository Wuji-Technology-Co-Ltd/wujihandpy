import pybind11_stubgen
import re


def main():
    pybind11_stubgen.main(["wujihandpy._core", "-o", "src"])

    post_process_async_function()


def post_process_async_function():
    with open("src/wujihandpy/_core/__init__.pyi", "r") as file:
        content = file.read()

    sections = content.split("class")
    for i in range(len(sections)):
        return_type_dict = {}
        matches = re.finditer(r"def (read|write)_(\S+?)\(.+?\) -> (\S+?):", sections[i])
        for match in matches:
            groups = match.groups()
            if not groups[1].endswith("async") and not groups[1].endswith("unchecked"):
                return_type_dict[f"{groups[0]}_{groups[1]}"] = groups[2]

        for key, value in return_type_dict.items():
            sections[i] = re.sub(
                rf"def {key}_async\((.+?)\) -> typing\.Any:",
                lambda match: f"async def {key}_async({match.group(1)}) -> {value}:",
                sections[i],
            )

    content = "class".join(sections)
    with open("src/wujihandpy/_core/__init__.pyi", "w") as file:
        file.write(content)


if __name__ == "__main__":
    main()
