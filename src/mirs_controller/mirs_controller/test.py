import asyncio

def do_something(self):
    for i in range(10e3):
        print(i)
async def test1(args):
    print("args :",args)
    do_something()

async def test2(args):
    print("args :",args)
    do_something()

async def main():
    print("Main running...")
    test1([1,2,9])
    print("Main3 running...")
    test2(['a','h'])


asyncio.run(main())
